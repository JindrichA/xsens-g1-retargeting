#!/usr/bin/env python3
"""Retarget recorded Xsens MVNX motion and visualize it on G1 in Isaac Sim."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time

from isaacsim import SimulationApp


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MVNX = ROOT / "xsens_TJ_08072026" / "MVNX export" / "Bending forward.mvnx"
PROJECTS_ROOT = Path(os.environ.get("RETARGET_PROJECTS_ROOT", ROOT.parent)).expanduser()
DEFAULT_G1_URDF = Path(
    os.environ.get(
        "RETARGET_G1_URDF",
        PROJECTS_ROOT / "unitree_rl_gym/resources/robots/g1_description/g1_29dof.urdf",
    )
).expanduser()
G1_USD_CACHE = ROOT / ".cache" / "isaac_g1"


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mvnx", type=Path, default=DEFAULT_MVNX)
parser.add_argument("--g1-urdf", type=Path, default=DEFAULT_G1_URDF)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--hide-xsens", action="store_true", help="Hide the synchronized Xsens skeleton")
parser.add_argument("--xsens-offset-y", type=float, default=1.5, help="Xsens skeleton lateral display offset in meters")
parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many 50 Hz frames; 0 plays the clip")
parser.add_argument("--loop", action="store_true", help="Loop until the Isaac window closes")
parser.add_argument("--scale", type=float, default=0.75, help="Human displacement to G1 displacement scale")
parser.add_argument("--ik-steps", type=int, default=4, help="Pink solver iterations per output frame")
parser.add_argument("--max-joint-speed", type=float, default=4.0, help="Retargeted joint speed limit in rad/s")
parser.add_argument("--no-realtime", action="store_true", help="Do not pace playback to 50 Hz")
parser.add_argument("--screenshot", type=Path, help="Capture the viewport after the first G1 pose")
parser.add_argument(
    "--dynamic",
    action="store_true",
    help="Enable gravity and dynamics without a balance controller (the G1 will fall)",
)
args, _ = parser.parse_known_args()

if args.scale <= 0:
    parser.error("--scale must be positive")
if args.ik_steps <= 0:
    parser.error("--ik-steps must be positive")
if args.max_joint_speed <= 0:
    parser.error("--max-joint-speed must be positive")
if args.max_frames < 0:
    parser.error("--max-frames cannot be negative")

simulation_app = SimulationApp({"headless": args.headless, "hide_ui": False})

import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot_motion.pink", True
)
simulation_app.update()

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
from isaacsim.robot_motion.pink import load_pink_robot
import omni.kit.renderer_capture
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
import omni.usd
import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask, PostureTask
from pxr import Gf, Usd, UsdGeom, Vt

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from retargeting.mvnx import MvnxFrame, iter_frames, read_metadata
from retargeting.xsens_skeleton import IK_SOURCE_SEGMENTS, display_positions, edge_indices


OUTPUT_RATE = 50
ROOT_HEIGHT = 0.76
TASK_SEGMENTS = {
    "torso_link": "T8",
    "left_wrist_yaw_link": "LeftHand",
    "right_wrist_yaw_link": "RightHand",
    "left_ankle_roll_link": "LeftFoot",
    "right_ankle_roll_link": "RightFoot",
}
LEFT_COLOR = (0.1, 0.7, 1.0, 1.0)
RIGHT_COLOR = (1.0, 0.45, 0.1, 1.0)
CENTER_COLOR = (0.85, 0.85, 0.85, 1.0)
CONTACT_COLOR = (0.2, 1.0, 0.25, 1.0)


def ensure_g1_usd(urdf_path: Path) -> Path:
    cached = G1_USD_CACHE / "g1_29dof" / "g1_29dof.usda"
    if cached.exists() and cached.stat().st_mtime >= urdf_path.stat().st_mtime:
        return cached
    G1_USD_CACHE.mkdir(parents=True, exist_ok=True)
    print(f"Importing visible G1 asset from {urdf_path}")
    config = URDFImporterConfig()
    config.urdf_path = str(urdf_path)
    config.usd_path = str(G1_USD_CACHE)
    config.fix_base = False
    config.merge_fixed_joints = False
    config.joint_target_type = "position"
    output = URDFImporter(config).import_urdf()
    if not output:
        raise RuntimeError(f"Isaac URDF importer failed for {urdf_path}")
    return Path(output)


def quaternion_matrix(quaternion: tuple[float, float, float, float]) -> np.ndarray:
    w, x, y, z = quaternion
    norm = np.linalg.norm([w, x, y, z])
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    return pin.Quaternion(w / norm, x / norm, y / norm, z / norm).toRotationMatrix()


def matrix_quaternion(matrix: np.ndarray) -> np.ndarray:
    quaternion = pin.Quaternion(matrix)
    quaternion.normalize()
    return np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])


def resample(path: Path) -> tuple[object, list[MvnxFrame]]:
    metadata = read_metadata(path)
    output: list[MvnxFrame] = []
    output_index = 0
    target_source_index = 0
    for frame in iter_frames(path, metadata):
        if frame.index < target_source_index:
            continue
        output.append(frame)
        output_index += 1
        target_source_index = round(output_index * metadata.frame_rate / OUTPUT_RATE)
    if not output:
        raise RuntimeError(f"no motion frames in {path}")
    return metadata, output


class XsensSkeletonDisplay:
    def __init__(self, metadata: object, initial_frame: MvnxFrame, offset_y: float):
        self.segment_names = metadata.segment_names
        self.segment_index = {name: index for index, name in enumerate(self.segment_names)}
        self.edges = edge_indices(self.segment_names)
        self.contact_index = {name: index for index, name in enumerate(metadata.contact_names)}
        pelvis_index = self.segment_index["Pelvis"]
        self.initial_pelvis_position = initial_frame.segment_positions[pelvis_index]
        self.initial_pelvis_quaternion = initial_frame.segment_quaternions[pelvis_index]
        # Offset partly along -X so the human and G1 remain at similar camera depth.
        self.display_origin = (-2.0 * offset_y / 3.0, offset_y, self.initial_pelvis_position[2])

        stage = omni.usd.get_context().get_stage()
        self.lines = UsdGeom.BasisCurves.Define(stage, "/World/XsensSkeleton/Segments")
        self.lines.CreateTypeAttr(UsdGeom.Tokens.linear)
        self.lines.CreateCurveVertexCountsAttr(Vt.IntArray([2] * len(self.edges)))
        self.lines.CreateWidthsAttr(Vt.FloatArray([0.018] * (2 * len(self.edges))))
        self.lines.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        line_colors = [
            self._color(self.segment_names[child])[:3]
            for _, child in self.edges
            for _ in range(2)
        ]
        line_colors_primvar = self.lines.GetDisplayColorPrimvar()
        line_colors_primvar.Set(Vt.Vec3fArray([Gf.Vec3f(*color) for color in line_colors]))
        line_colors_primvar.SetInterpolation(UsdGeom.Tokens.vertex)

        self.points = UsdGeom.Points.Define(stage, "/World/XsensSkeleton/Joints")
        self.points.CreateWidthsAttr(
            Vt.FloatArray([0.075 if name in IK_SOURCE_SEGMENTS else 0.045 for name in self.segment_names])
        )
        self.points.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        self.point_colors_primvar = self.points.GetDisplayColorPrimvar()
        self.point_colors_primvar.SetInterpolation(UsdGeom.Tokens.vertex)

    @staticmethod
    def _color(name: str) -> tuple[float, float, float, float]:
        if name.startswith("Left"):
            return LEFT_COLOR
        if name.startswith("Right"):
            return RIGHT_COLOR
        return CENTER_COLOR

    def _foot_in_contact(self, frame: MvnxFrame, side: str) -> bool:
        indexes = [
            index
            for name, index in self.contact_index.items()
            if name.startswith(f"{side}Foot_")
        ]
        return any(frame.foot_contacts[index] for index in indexes)

    def update(self, frame: MvnxFrame) -> None:
        points = display_positions(
            frame.segment_positions,
            self.initial_pelvis_position,
            self.initial_pelvis_quaternion,
            self.display_origin,
        )
        line_points = [points[index] for edge in self.edges for index in edge]
        self.lines.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*point) for point in line_points]))
        self.points.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*point) for point in points]))

        left_contact = self._foot_in_contact(frame, "Left")
        right_contact = self._foot_in_contact(frame, "Right")
        point_colors = []
        for name in self.segment_names:
            contact = (left_contact and name in {"LeftFoot", "LeftToe"}) or (
                right_contact and name in {"RightFoot", "RightToe"}
            )
            color = CONTACT_COLOR if contact else self._color(name)
            point_colors.append(color[:3])
        self.point_colors_primvar.Set(Vt.Vec3fArray([Gf.Vec3f(*color) for color in point_colors]))


class G1Retargeter:
    def __init__(self, urdf_path: Path, metadata: object, initial_frame: MvnxFrame, scale: float):
        self.robot = load_pink_robot(urdf_path=urdf_path)
        self.model = self.robot.model
        self.scale = scale
        self.segment_index = {name: index for index, name in enumerate(metadata.segment_names)}
        missing = sorted(set(TASK_SEGMENTS.values()) - self.segment_index.keys())
        if missing:
            raise RuntimeError(f"MVNX is missing required segments: {', '.join(missing)}")

        q0 = pin.neutral(self.model)
        initial_angles = {
            "left_hip_pitch_joint": -0.312,
            "right_hip_pitch_joint": -0.312,
            "left_knee_joint": 0.669,
            "right_knee_joint": 0.669,
            "left_ankle_pitch_joint": -0.363,
            "right_ankle_pitch_joint": -0.363,
            "left_shoulder_roll_joint": 0.2,
            "right_shoulder_roll_joint": -0.2,
            "left_elbow_joint": 0.6,
            "right_elbow_joint": 0.6,
        }
        for name, angle in initial_angles.items():
            joint_id = self.model.getJointId(name)
            q0[self.model.joints[joint_id].idx_q] = angle
        self.robot.q0 = q0.copy()
        self.configuration = pink.Configuration(self.model, self.robot.data, q0)
        self.last_output_q = q0.copy()

        self.tasks: dict[str, FrameTask] = {}
        for frame_name in TASK_SEGMENTS:
            if "wrist" in frame_name:
                costs = {"position_cost": 8.0, "orientation_cost": 1.5, "lm_damping": 2.0}
            elif "ankle" in frame_name:
                costs = {"position_cost": 10.0, "orientation_cost": 2.0, "lm_damping": 2.0}
            else:
                costs = {"position_cost": 3.0, "orientation_cost": 3.0, "lm_damping": 1.0}
            task = FrameTask(frame_name, **costs)
            task.set_target_from_configuration(self.configuration)
            self.tasks[frame_name] = task
        self.posture = PostureTask(cost=0.03, lm_damping=1.0)
        self.posture.set_target(q0)

        pelvis_index = self.segment_index["Pelvis"]
        self.initial_pelvis_position = np.asarray(initial_frame.segment_positions[pelvis_index])
        self.initial_pelvis_rotation = quaternion_matrix(initial_frame.segment_quaternions[pelvis_index])
        self.initial_human_relative: dict[str, pin.SE3] = {}
        self.initial_robot_frames: dict[str, pin.SE3] = {}
        for robot_frame, human_segment in TASK_SEGMENTS.items():
            self.initial_human_relative[robot_frame] = self._human_relative(initial_frame, human_segment)
            self.initial_robot_frames[robot_frame] = self.configuration.get_transform_frame_to_world(robot_frame)

    def _human_relative(self, frame: MvnxFrame, segment_name: str) -> pin.SE3:
        pelvis_index = self.segment_index["Pelvis"]
        segment_index = self.segment_index[segment_name]
        pelvis_position = np.asarray(frame.segment_positions[pelvis_index])
        pelvis_rotation = quaternion_matrix(frame.segment_quaternions[pelvis_index])
        segment_position = np.asarray(frame.segment_positions[segment_index])
        segment_rotation = quaternion_matrix(frame.segment_quaternions[segment_index])
        return pin.SE3(
            pelvis_rotation.T @ segment_rotation,
            pelvis_rotation.T @ (segment_position - pelvis_position),
        )

    def root_pose(self, frame: MvnxFrame) -> tuple[np.ndarray, np.ndarray]:
        pelvis_index = self.segment_index["Pelvis"]
        position = np.asarray(frame.segment_positions[pelvis_index])
        rotation = quaternion_matrix(frame.segment_quaternions[pelvis_index])
        delta_position = self.initial_pelvis_rotation.T @ (position - self.initial_pelvis_position)
        world_position = np.array(
            [self.scale * delta_position[0], self.scale * delta_position[1], ROOT_HEIGHT + self.scale * delta_position[2]]
        )
        world_rotation = rotation @ self.initial_pelvis_rotation.T
        return world_position, matrix_quaternion(world_rotation)

    def solve(self, frame: MvnxFrame, iterations: int, max_joint_speed: float) -> np.ndarray:
        for robot_frame, human_segment in TASK_SEGMENTS.items():
            current = self._human_relative(frame, human_segment)
            human_initial = self.initial_human_relative[robot_frame]
            robot_initial = self.initial_robot_frames[robot_frame]
            target_rotation = robot_initial.rotation @ human_initial.rotation.T @ current.rotation
            target_translation = robot_initial.translation + self.scale * (
                current.translation - human_initial.translation
            )
            self.tasks[robot_frame].set_target(pin.SE3(target_rotation, target_translation))

        all_tasks = [*self.tasks.values(), self.posture]
        for _ in range(iterations):
            velocity = pink.solve_ik(
                self.configuration,
                all_tasks,
                dt=1.0 / OUTPUT_RATE,
                solver="osqp",
                safety_break=False,
            )
            self.configuration.integrate_inplace(velocity, 1.0 / OUTPUT_RATE)
            self.configuration.q = np.clip(
                self.configuration.q,
                self.model.lowerPositionLimit,
                self.model.upperPositionLimit,
            )
            self.configuration.update()
        if not np.isfinite(self.configuration.q).all():
            raise RuntimeError(f"IK produced non-finite values at source frame {frame.index}")
        max_step = max_joint_speed / OUTPUT_RATE
        output_q = self.last_output_q + np.clip(
            self.configuration.q - self.last_output_q,
            -max_step,
            max_step,
        )
        self.configuration.q = output_q.copy()
        self.configuration.update()
        self.last_output_q = output_q.copy()
        return output_q


def main() -> int:
    for required in (args.mvnx, args.g1_urdf):
        if not required.exists():
            raise FileNotFoundError(required)
    g1_usd = ensure_g1_usd(args.g1_urdf)

    metadata, frames = resample(args.mvnx)
    if args.max_frames:
        frames = frames[: args.max_frames]
    print(
        f"Loaded {args.mvnx.name}: {len(frames)} output frames at {OUTPUT_RATE} Hz "
        f"from {metadata.frame_rate} Hz Xsens data"
    )

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage_utils.add_reference_to_stage(usd_path=str(g1_usd), path="/World/G1")
    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(1000)
    camera_target_x = 0.0 if args.hide_xsens else -args.xsens_offset_y / 3.0
    camera_target_y = 0.0 if args.hide_xsens else args.xsens_offset_y / 2.0
    set_camera_view(
        eye=[camera_target_x + 3.2, camera_target_y + 2.75, 1.8],
        target=[camera_target_x, camera_target_y, 0.85],
        camera_prim_path="/OmniverseKit_Persp",
    )
    g1 = Articulation("/World/G1")
    xsens_display = None if args.hide_xsens else XsensSkeletonDisplay(metadata, frames[0], args.xsens_offset_y)
    if xsens_display:
        xsens_display.update(frames[0])

    app_utils.play()
    for _ in range(5):
        simulation_app.update()

    if args.dynamic:
        print("Playback mode: DYNAMIC (gravity enabled, no balance controller; falling is expected)")
    else:
        physics_scenes = SimulationManager.get_physics_scenes()
        if not physics_scenes:
            raise RuntimeError("Isaac did not create a physics scene")
        for physics_scene in physics_scenes:
            physics_scene.set_gravity((0.0, 0.0, 0.0))
        print("Playback mode: KINEMATIC (zero gravity; pose and velocities reset at 50 Hz)")

    retargeter = G1Retargeter(args.g1_urdf, metadata, frames[0], args.scale)
    if xsens_display:
        print("Xsens skeleton: visible beside G1 (left cyan, right orange, foot contact green)")
    isaac_indices = {name: index for index, name in enumerate(g1.dof_names)}
    missing_joints = sorted(set(retargeter.robot.controlled_joint_names) - isaac_indices.keys())
    if missing_joints:
        raise RuntimeError(f"Isaac G1 is missing IK joints: {', '.join(missing_joints)}")
    controlled_indices = [isaac_indices[name] for name in retargeter.robot.controlled_joint_names]
    isaac_positions = np.zeros(len(g1.dof_names), dtype=np.float32)
    zero_dof_velocities = np.zeros((1, len(g1.dof_names)), dtype=np.float32)
    zero_root_velocities = np.zeros((1, 3), dtype=np.float32)

    played = 0
    maximum_joint_step = 0.0
    previous_q: np.ndarray | None = None
    try:
        while simulation_app.is_running():
            for frame in frames:
                started = time.monotonic()
                q = retargeter.solve(frame, args.ik_steps, args.max_joint_speed)
                if previous_q is not None:
                    maximum_joint_step = max(maximum_joint_step, float(np.max(np.abs(q - previous_q))))
                previous_q = q
                isaac_positions[controlled_indices] = q
                root_position, root_orientation = retargeter.root_pose(frame)
                if xsens_display:
                    xsens_display.update(frame)
                g1.set_dof_positions(isaac_positions.reshape(1, -1))
                if not args.dynamic:
                    g1.set_dof_velocities(zero_dof_velocities)
                g1.set_world_poses(
                    positions=root_position.reshape(1, 3),
                    orientations=root_orientation.reshape(1, 4),
                )
                if not args.dynamic:
                    g1.set_velocities(zero_root_velocities, zero_root_velocities)
                if played == 0:
                    # Referenced USD layers can author viewport state, so frame the
                    # robot only after its first world pose has been applied.
                    set_camera_view(
                        eye=[camera_target_x + 3.2, camera_target_y + 2.75, 1.8],
                        target=[camera_target_x, camera_target_y, 0.85],
                        camera_prim_path="/OmniverseKit_Persp",
                    )
                simulation_app.update()
                if played == 0:
                    stage = omni.usd.get_context().get_stage()
                    robot_prim = stage.GetPrimAtPath("/World/G1")
                    bounds = UsdGeom.BBoxCache(
                        Usd.TimeCode.Default(),
                        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                    ).ComputeWorldBound(robot_prim).ComputeAlignedRange()
                    print(f"G1 visible world bounds: min={bounds.GetMin()}, max={bounds.GetMax()}")
                    camera_prim = stage.GetPrimAtPath("/OmniverseKit_Persp")
                    camera_position = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ).ExtractTranslation()
                    print(
                        f"Stage meters-per-unit: {UsdGeom.GetStageMetersPerUnit(stage)}; "
                        f"camera position: {camera_position}"
                    )
                    if args.screenshot:
                        screenshot = args.screenshot.expanduser().resolve()
                        screenshot.parent.mkdir(parents=True, exist_ok=True)
                        capture_viewport_to_file(get_active_viewport(), file_path=str(screenshot))
                        for _ in range(30):
                            simulation_app.update()
                        omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
                        print(f"Captured viewport: {screenshot}")
                played += 1
                if not args.no_realtime:
                    remaining = 1.0 / OUTPUT_RATE - (time.monotonic() - started)
                    if remaining > 0:
                        time.sleep(remaining)
                if not simulation_app.is_running():
                    break
            if args.headless:
                break
            if not args.loop:
                print("Playback complete; keeping the final G1 pose visible. Close Isaac Sim to exit.")
                app_utils.pause()
                while simulation_app.is_running():
                    simulation_app.update()
                    time.sleep(0.02)
                break
    finally:
        print(f"Played {played} frames; maximum IK joint step: {maximum_joint_step:.4f} rad")
        app_utils.stop()
        simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

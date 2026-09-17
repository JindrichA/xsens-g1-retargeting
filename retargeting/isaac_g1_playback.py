#!/usr/bin/env python3
"""Retarget recorded Xsens MVNX motion and visualize it on G1 in Isaac Sim."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time

SCRIPT_ROOT = Path(__file__).resolve().parent.parent
if __package__:
    from .paths import PROJECT_ROOT as ROOT, projects_root
else:
    sys.path.insert(0, str(SCRIPT_ROOT))
    from retargeting.paths import PROJECT_ROOT as ROOT, projects_root

from isaacsim import SimulationApp


DEFAULT_MVNX = ROOT / "xsens_TJ_08072026" / "MVNX export" / "Bending forward.mvnx"
PROJECTS_ROOT = projects_root()
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
parser.add_argument("--hide-diagnostics", action="store_true", help="Hide desired/achieved IK markers")
parser.add_argument("--xsens-offset-y", type=float, default=1.5, help="Xsens skeleton lateral display offset in meters")
parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many 50 Hz frames; 0 plays the clip")
parser.add_argument("--loop", action="store_true", help="Loop until the Isaac window closes")
parser.add_argument("--scale", type=float, default=0.75, help="Human displacement to G1 displacement scale")
parser.add_argument("--torso-scale", type=float, help="Override displacement scale for the torso task")
parser.add_argument("--arm-scale", type=float, help="Override displacement scale for both wrist tasks")
parser.add_argument("--leg-scale", type=float, help="Override displacement scale for both ankle tasks")
parser.add_argument("--ik-steps", type=int, default=4, help="Pink solver iterations per output frame")
parser.add_argument("--max-joint-speed", type=float, default=4.0, help="Retargeted joint speed limit in rad/s")
parser.add_argument("--no-realtime", action="store_true", help="Do not pace playback to 50 Hz")
parser.add_argument("--screenshot", type=Path, help="Capture the viewport at --screenshot-frame")
parser.add_argument("--screenshot-frame", type=int, default=0, help="Zero-based output frame to capture")
parser.add_argument("--diagnostics-json", type=Path, help="Write per-frame diagnostics and summary as JSON")
parser.add_argument("--diagnostics-csv", type=Path, help="Write per-frame diagnostics as CSV")
parser.add_argument("--trajectory-dir", type=Path, help="Write a SONIC-compatible 50 Hz G1 reference directory")
parser.add_argument(
    "--dynamic",
    action="store_true",
    help="Enable gravity and dynamics without a balance controller (the G1 will fall)",
)
args, _ = parser.parse_known_args()

if any(
    value is not None and (not math.isfinite(value) or value <= 0)
    for value in (args.scale, args.torso_scale, args.arm_scale, args.leg_scale)
):
    parser.error("all displacement scales must be finite and positive")
if args.ik_steps <= 0:
    parser.error("--ik-steps must be positive")
if args.max_joint_speed <= 0:
    parser.error("--max-joint-speed must be positive")
if args.max_frames < 0:
    parser.error("--max-frames cannot be negative")
if args.screenshot_frame < 0:
    parser.error("--screenshot-frame cannot be negative")

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
from pxr import Gf, Usd, UsdGeom, Vt

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from retargeting.diagnostics import DiagnosticsRecorder
from retargeting.g1_retargeter import G1Retargeter, quaternion_matrix
from retargeting.mvnx import MvnxFrame, iter_frames, read_metadata
from retargeting.retargeting_config import OUTPUT_RATE, TASK_LABELS, TASK_SEGMENTS, RetargetingConfig
from retargeting.trajectory import TrajectoryRecorder
from retargeting.xsens_skeleton import IK_SOURCE_SEGMENTS, display_positions, edge_indices


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


class RetargetingDiagnosticsDisplay:
    def __init__(self, frame_names: tuple[str, ...]):
        self.frame_names = frame_names
        stage = omni.usd.get_context().get_stage()

        self.targets = UsdGeom.Points.Define(stage, "/World/RetargetingDiagnostics/Targets")
        self.targets.CreateWidthsAttr(Vt.FloatArray([0.055] * len(frame_names)))
        self.targets.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        target_colors = self.targets.GetDisplayColorPrimvar()
        target_colors.Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.1, 0.1)] * len(frame_names)))
        target_colors.SetInterpolation(UsdGeom.Tokens.vertex)
        self.targets.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0)] * len(frame_names)))

        self.achieved = UsdGeom.Points.Define(stage, "/World/RetargetingDiagnostics/Achieved")
        self.achieved.CreateWidthsAttr(Vt.FloatArray([0.08] * len(frame_names)))
        self.achieved.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        achieved_colors = self.achieved.GetDisplayColorPrimvar()
        achieved_colors.Set(Vt.Vec3fArray([Gf.Vec3f(0.1, 1.0, 0.1)] * len(frame_names)))
        achieved_colors.SetInterpolation(UsdGeom.Tokens.vertex)
        self.achieved.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0)] * len(frame_names)))

        self.errors = UsdGeom.BasisCurves.Define(stage, "/World/RetargetingDiagnostics/Errors")
        self.errors.CreateTypeAttr(UsdGeom.Tokens.linear)
        self.errors.CreateCurveVertexCountsAttr(Vt.IntArray([2] * len(frame_names)))
        self.errors.CreateWidthsAttr(Vt.FloatArray([0.012] * (2 * len(frame_names))))
        self.errors.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        error_colors = self.errors.GetDisplayColorPrimvar()
        error_colors.Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 1.0, 0.1)] * (2 * len(frame_names))))
        error_colors.SetInterpolation(UsdGeom.Tokens.vertex)
        self.errors.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0)] * (2 * len(frame_names))))

    def update(
        self,
        targets: dict[str, pin.SE3],
        achieved: dict[str, pin.SE3],
        root_position: np.ndarray,
        root_orientation: np.ndarray,
    ) -> None:
        root_rotation = quaternion_matrix(tuple(root_orientation))

        def world_position(transform: pin.SE3) -> tuple[float, float, float]:
            position = root_position + root_rotation @ transform.translation
            return tuple(float(value) for value in position)

        target_points = [world_position(targets[name]) for name in self.frame_names]
        achieved_points = [world_position(achieved[name]) for name in self.frame_names]
        error_points = [point for pair in zip(target_points, achieved_points) for point in pair]
        self.targets.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*point) for point in target_points]))
        self.achieved.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*point) for point in achieved_points]))
        self.errors.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*point) for point in error_points]))


def print_diagnostics_summary(recorder: DiagnosticsRecorder) -> None:
    summary = recorder.summary()
    print("IK tracking diagnostics (after joint-speed limiting):")
    for name in recorder.task_names:
        task = summary["tasks"][name]
        print(
            f"  {name}: position RMS {task['rms_position_cm']:.2f} cm, "
            f"max {task['max_position_cm']:.2f} cm; orientation RMS "
            f"{task['rms_orientation_deg']:.1f} deg, max {task['max_orientation_deg']:.1f} deg"
        )
    print(
        f"  constraints: rate-limited in {summary['rate_limited_frame_count']}/{summary['frame_count']} frames; "
        f"near a joint limit in {summary['near_limit_frame_count']}/{summary['frame_count']} frames"
    )
    for title, key in (
        ("most rate-limited", "rate_limited_joint_counts"),
        ("most near-limit", "near_limit_joint_counts"),
    ):
        counts = sorted(summary[key].items(), key=lambda item: (-item[1], item[0]))[:5]
        if counts:
            print(f"  {title}: {', '.join(f'{name} ({count})' for name, count in counts)}")


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
    pink_robot = load_pink_robot(urdf_path=args.g1_urdf)
    retargeting_config = RetargetingConfig(
        scale=args.scale,
        torso_scale=args.torso_scale,
        arm_scale=args.arm_scale,
        leg_scale=args.leg_scale,
        iterations=args.ik_steps,
        max_joint_speed=args.max_joint_speed,
        solver="osqp",
    )
    retargeter = G1Retargeter(
        args.g1_urdf,
        metadata.segment_names,
        frames[0],
        retargeting_config,
        model=pink_robot.model,
        data=pink_robot.data,
        controlled_joint_names=pink_robot.controlled_joint_names,
    )
    pink_robot.q0 = retargeter.last_output_q.copy()
    recorder = DiagnosticsRecorder(tuple(TASK_LABELS.values()))
    trajectory = TrajectoryRecorder(
        str(args.mvnx), OUTPUT_RATE, retargeting_config.to_dict()
    )
    xsens_display = None if args.hide_xsens else XsensSkeletonDisplay(metadata, frames[0], args.xsens_offset_y)
    if xsens_display:
        xsens_display.update(frames[0])
    diagnostics_display = None if args.hide_diagnostics else RetargetingDiagnosticsDisplay(tuple(TASK_SEGMENTS))

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

    if xsens_display:
        print("Xsens skeleton: visible beside G1 (left cyan, right orange, foot contact green)")
    if diagnostics_display:
        print("IK diagnostics: desired targets red, achieved frames green, position error lines yellow")
    isaac_indices = {name: index for index, name in enumerate(g1.dof_names)}
    missing_joints = sorted(set(retargeter.controlled_joint_names) - isaac_indices.keys())
    if missing_joints:
        raise RuntimeError(f"Isaac G1 is missing IK joints: {', '.join(missing_joints)}")
    controlled_indices = [isaac_indices[name] for name in retargeter.controlled_joint_names]
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
                result = retargeter.solve(frame)
                if played < len(frames):
                    recorder.add(result.diagnostics)
                joint_positions = result.joint_positions
                controlled_q = np.asarray(
                    [joint_positions[name] for name in retargeter.controlled_joint_names]
                )
                if previous_q is not None:
                    maximum_joint_step = max(maximum_joint_step, float(np.max(np.abs(controlled_q - previous_q))))
                previous_q = controlled_q
                isaac_positions[controlled_indices] = controlled_q
                root_position, root_orientation = retargeter.root_pose(frame)
                if played < len(frames):
                    trajectory.add(frame.index, joint_positions, root_position, root_orientation)
                if xsens_display:
                    xsens_display.update(frame)
                if diagnostics_display:
                    diagnostics_display.update(
                        result.targets,
                        result.achieved,
                        root_position,
                        root_orientation,
                    )
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
                if args.screenshot and played == args.screenshot_frame:
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
        try:
            print(f"Played {played} frames; maximum IK joint step: {maximum_joint_step:.4f} rad")
            if recorder.frames:
                print_diagnostics_summary(recorder)
                if args.diagnostics_json:
                    print(f"Wrote diagnostics JSON: {recorder.write_json(args.diagnostics_json)}")
                if args.diagnostics_csv:
                    print(f"Wrote diagnostics CSV: {recorder.write_csv(args.diagnostics_csv)}")
                if args.trajectory_dir:
                    print(f"Wrote SONIC trajectory: {trajectory.write(args.trajectory_dir)}")
        finally:
            app_utils.stop()
            simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

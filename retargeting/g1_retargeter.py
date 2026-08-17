"""Headless Pink retargeter shared by recorded and live Xsens pipelines."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask, PostureTask

from .live_frame import CANONICAL_SEGMENTS, LiveFrame
from .mvnx import MvnxFrame
from .trajectory import SONIC_G1_JOINT_ORDER


OUTPUT_RATE = 50
ROOT_HEIGHT = 0.76
TASK_SEGMENTS = {
    "torso_link": "T8",
    "left_wrist_yaw_link": "LeftHand",
    "right_wrist_yaw_link": "RightHand",
    "left_ankle_roll_link": "LeftFoot",
    "right_ankle_roll_link": "RightFoot",
}
STANDING_ANGLES = {
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


def quaternion_matrix(quaternion) -> np.ndarray:
    w, x, y, z = quaternion
    norm = np.linalg.norm([w, x, y, z])
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    return pin.Quaternion(w / norm, x / norm, y / norm, z / norm).toRotationMatrix()


def matrix_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    quaternion = pin.Quaternion(matrix)
    quaternion.normalize()
    return (float(quaternion.w), float(quaternion.x), float(quaternion.y), float(quaternion.z))


def mvnx_from_live(frame: LiveFrame, index: int) -> MvnxFrame:
    return MvnxFrame(
        index=index,
        time_seconds=index / OUTPUT_RATE,
        source_time_ms=frame.source_frame_time_ms,
        segment_quaternions=tuple(frame.segments[name].quaternion_wxyz for name in CANONICAL_SEGMENTS),
        segment_positions=tuple(frame.segments[name].position for name in CANONICAL_SEGMENTS),
        joint_angles_zxy=(),
        foot_contacts=(),
        center_of_mass=(),
    )


class G1Retargeter:
    def __init__(self, urdf_path: Path, segment_names, initial_frame: MvnxFrame, scale: float = 0.75):
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.scale = scale
        self.segment_index = {name: index for index, name in enumerate(segment_names)}
        missing = sorted((set(TASK_SEGMENTS.values()) | {"Pelvis"}) - self.segment_index.keys())
        if missing:
            raise RuntimeError(f"Xsens frame is missing required segments: {', '.join(missing)}")

        q0 = pin.neutral(self.model)
        for name, angle in STANDING_ANGLES.items():
            joint = self.model.joints[self.model.getJointId(name)]
            q0[joint.idx_q] = angle
        self.configuration = pink.Configuration(self.model, self.data, q0)
        self.last_output_q = q0.copy()
        self.controlled_joint_indices = {}
        for name in SONIC_G1_JOINT_ORDER:
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1:
                raise RuntimeError(f"controlled joint {name} has unsupported configuration size {joint.nq}")
            self.controlled_joint_indices[name] = joint.idx_q

        self.tasks = {}
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
        self.initial_human_relative = {}
        self.initial_robot_frames = {}
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

    def root_quaternion(self, frame: MvnxFrame) -> tuple[float, float, float, float]:
        pelvis_index = self.segment_index["Pelvis"]
        rotation = quaternion_matrix(frame.segment_quaternions[pelvis_index])
        return matrix_quaternion(rotation @ self.initial_pelvis_rotation.T)

    def solve(self, frame: MvnxFrame, iterations: int = 4, max_joint_speed: float = 4.0) -> tuple[list[float], list[float]]:
        for robot_frame, human_segment in TASK_SEGMENTS.items():
            current = self._human_relative(frame, human_segment)
            human_initial = self.initial_human_relative[robot_frame]
            robot_initial = self.initial_robot_frames[robot_frame]
            self.tasks[robot_frame].set_target(
                pin.SE3(
                    robot_initial.rotation @ human_initial.rotation.T @ current.rotation,
                    robot_initial.translation + self.scale * (current.translation - human_initial.translation),
                )
            )
        for _ in range(iterations):
            velocity = pink.solve_ik(
                self.configuration,
                [*self.tasks.values(), self.posture],
                dt=1.0 / OUTPUT_RATE,
                solver="quadprog",
                safety_break=False,
            )
            self.configuration.integrate_inplace(velocity, 1.0 / OUTPUT_RATE)
            self.configuration.q = np.clip(
                self.configuration.q, self.model.lowerPositionLimit, self.model.upperPositionLimit
            )
            self.configuration.update()
        if not np.isfinite(self.configuration.q).all():
            raise RuntimeError("live IK produced non-finite values")
        max_step = max_joint_speed / OUTPUT_RATE
        requested = self.configuration.q - self.last_output_q
        output = self.last_output_q + np.clip(requested, -max_step, max_step)
        velocities = (output - self.last_output_q) * OUTPUT_RATE
        self.configuration.q = output.copy()
        self.configuration.update()
        self.last_output_q = output.copy()
        return (
            [float(output[self.controlled_joint_indices[name]]) for name in SONIC_G1_JOINT_ORDER],
            [float(velocities[self.controlled_joint_indices[name]]) for name in SONIC_G1_JOINT_ORDER],
        )

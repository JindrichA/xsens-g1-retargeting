"""Shared Pink retargeter for recorded, mock, and live Xsens frames."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask, PostureTask

from .diagnostics import FrameDiagnostics
from .live_frame import CANONICAL_SEGMENTS, LiveFrame
from .mvnx import MvnxFrame
from .retargeting_config import (
    OUTPUT_RATE,
    ROOT_HEIGHT,
    STANDING_ANGLES,
    TASK_LABELS,
    TASK_SEGMENTS,
    RetargetingConfig,
)
from .trajectory import SONIC_G1_JOINT_ORDER


@dataclass(frozen=True)
class RetargetResult:
    configuration: np.ndarray
    joint_positions: dict[str, float]
    joint_velocities: dict[str, float]
    diagnostics: FrameDiagnostics
    targets: dict[str, pin.SE3]
    achieved: dict[str, pin.SE3]


def quaternion_matrix(quaternion: Sequence[float]) -> np.ndarray:
    w, x, y, z = quaternion
    norm = np.linalg.norm([w, x, y, z])
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    return pin.Quaternion(w / norm, x / norm, y / norm, z / norm).toRotationMatrix()


def matrix_quaternion(matrix: np.ndarray) -> np.ndarray:
    quaternion = pin.Quaternion(matrix)
    quaternion.normalize()
    return np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])


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
    def __init__(
        self,
        urdf_path: Path,
        segment_names: Sequence[str],
        initial_frame: MvnxFrame,
        config: RetargetingConfig | None = None,
        *,
        model: Any = None,
        data: Any = None,
        controlled_joint_names: Sequence[str] = SONIC_G1_JOINT_ORDER,
    ) -> None:
        self.config = config or RetargetingConfig()
        self.model = model if model is not None else pin.buildModelFromUrdf(str(urdf_path))
        self.data = data if data is not None else self.model.createData()
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
        self.controlled_joint_names = tuple(controlled_joint_names)
        self.controlled_joint_indices = []
        for name in self.controlled_joint_names:
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1:
                raise RuntimeError(f"controlled joint {name} has unsupported configuration size {joint.nq}")
            self.controlled_joint_indices.append((name, joint.idx_q))

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

    def root_pose(self, frame: MvnxFrame) -> tuple[np.ndarray, np.ndarray]:
        pelvis_index = self.segment_index["Pelvis"]
        position = np.asarray(frame.segment_positions[pelvis_index])
        rotation = quaternion_matrix(frame.segment_quaternions[pelvis_index])
        delta_position = self.initial_pelvis_rotation.T @ (position - self.initial_pelvis_position)
        world_position = np.array(
            [
                self.config.scale * delta_position[0],
                self.config.scale * delta_position[1],
                ROOT_HEIGHT + self.config.scale * delta_position[2],
            ]
        )
        return world_position, matrix_quaternion(rotation @ self.initial_pelvis_rotation.T)

    def root_quaternion(self, frame: MvnxFrame) -> tuple[float, float, float, float]:
        _position, quaternion = self.root_pose(frame)
        return tuple(float(value) for value in quaternion)

    def solve(self, frame: MvnxFrame) -> RetargetResult:
        targets = {}
        for robot_frame, human_segment in TASK_SEGMENTS.items():
            current = self._human_relative(frame, human_segment)
            human_initial = self.initial_human_relative[robot_frame]
            robot_initial = self.initial_robot_frames[robot_frame]
            target = pin.SE3(
                robot_initial.rotation @ human_initial.rotation.T @ current.rotation,
                robot_initial.translation
                + self.config.scale_for(robot_frame)
                * (current.translation - human_initial.translation),
            )
            targets[robot_frame] = target
            self.tasks[robot_frame].set_target(target)

        for _ in range(self.config.iterations):
            velocity = pink.solve_ik(
                self.configuration,
                [*self.tasks.values(), self.posture],
                dt=1.0 / self.config.output_rate,
                solver=self.config.solver,
                safety_break=False,
            )
            self.configuration.integrate_inplace(velocity, 1.0 / self.config.output_rate)
            self.configuration.q = np.clip(
                self.configuration.q,
                self.model.lowerPositionLimit,
                self.model.upperPositionLimit,
            )
            self.configuration.update()
        if not np.isfinite(self.configuration.q).all():
            raise RuntimeError(f"IK produced non-finite values at source frame {frame.index}")

        max_step = self.config.max_joint_speed / self.config.output_rate
        requested_step = self.configuration.q - self.last_output_q
        rate_limited = tuple(
            name
            for name, index in self.controlled_joint_indices
            if abs(requested_step[index]) > max_step + 1e-9
        )
        output_q = self.last_output_q + np.clip(requested_step, -max_step, max_step)
        velocity_q = (output_q - self.last_output_q) * self.config.output_rate
        self.configuration.q = output_q.copy()
        self.configuration.update()
        self.last_output_q = output_q.copy()

        near_limit = []
        for name, index in self.controlled_joint_indices:
            margin = min(
                output_q[index] - self.model.lowerPositionLimit[index],
                self.model.upperPositionLimit[index] - output_q[index],
            )
            if math.isfinite(float(margin)) and margin <= self.config.near_limit_margin:
                near_limit.append(name)

        achieved = {
            robot_frame: self.configuration.get_transform_frame_to_world(robot_frame)
            for robot_frame in TASK_SEGMENTS
        }
        position_errors = {}
        orientation_errors = {}
        for robot_frame in TASK_SEGMENTS:
            label = TASK_LABELS[robot_frame]
            position_errors[label] = float(
                np.linalg.norm(achieved[robot_frame].translation - targets[robot_frame].translation)
            )
            rotation_error = targets[robot_frame].rotation.T @ achieved[robot_frame].rotation
            orientation_errors[label] = float(np.linalg.norm(pin.log3(rotation_error)))
        diagnostics = FrameDiagnostics(
            source_frame=frame.index,
            time_seconds=frame.time_seconds,
            position_errors_m=position_errors,
            orientation_errors_rad=orientation_errors,
            rate_limited_joints=rate_limited,
            near_limit_joints=tuple(near_limit),
        )
        joint_positions = {name: float(output_q[index]) for name, index in self.controlled_joint_indices}
        joint_velocities = {name: float(velocity_q[index]) for name, index in self.controlled_joint_indices}
        return RetargetResult(
            output_q,
            joint_positions,
            joint_velocities,
            diagnostics,
            targets,
            achieved,
        )

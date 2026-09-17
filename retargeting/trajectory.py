"""Dependency-free writer for SONIC-compatible G1 reference trajectories."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


# GR00T-WholeBodyControl gear_sonic/envs/env_utils/joint_utils.py.
SONIC_G1_JOINT_ORDER = (
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)


class TrajectoryRecorder:
    def __init__(
        self,
        source: str,
        rate_hz: int = 50,
        retargeting_config: Mapping[str, object] | None = None,
    ):
        if rate_hz <= 0:
            raise ValueError("trajectory rate must be positive")
        self.source = source
        self.rate_hz = rate_hz
        self.retargeting_config = dict(retargeting_config) if retargeting_config else None
        self.source_frames: list[int] = []
        self.joint_positions: list[tuple[float, ...]] = []
        self.joint_velocities: list[tuple[float, ...]] = []
        self.root_positions: list[tuple[float, float, float]] = []
        self.root_quaternions_wxyz: list[tuple[float, float, float, float]] = []

    def add(
        self,
        source_frame: int,
        joint_positions: Mapping[str, float],
        root_position: Sequence[float],
        root_quaternion_wxyz: Sequence[float],
    ) -> None:
        missing = sorted(set(SONIC_G1_JOINT_ORDER) - joint_positions.keys())
        if missing:
            raise ValueError(f"trajectory is missing G1 joints: {', '.join(missing)}")
        positions = tuple(float(joint_positions[name]) for name in SONIC_G1_JOINT_ORDER)
        root = tuple(float(value) for value in root_position)
        quaternion = tuple(float(value) for value in root_quaternion_wxyz)
        if len(root) != 3 or len(quaternion) != 4:
            raise ValueError("root position must have 3 values and quaternion must have 4")
        if not all(math.isfinite(value) for value in (*positions, *root, *quaternion)):
            raise ValueError("trajectory contains non-finite values")
        if self.joint_positions:
            previous = self.joint_positions[-1]
            velocities = tuple((current - old) * self.rate_hz for current, old in zip(positions, previous))
        else:
            velocities = (0.0,) * len(SONIC_G1_JOINT_ORDER)
        self.source_frames.append(source_frame)
        self.joint_positions.append(positions)
        self.joint_velocities.append(velocities)
        self.root_positions.append(root)
        self.root_quaternions_wxyz.append(quaternion)

    def write(self, directory: str | Path) -> Path:
        if not self.joint_positions:
            raise ValueError("cannot write an empty trajectory")
        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        _write_csv(
            directory / "joint_pos.csv",
            [f"joint_{index}" for index in range(29)],
            self.joint_positions,
        )
        _write_csv(
            directory / "joint_vel.csv",
            [f"joint_vel_{index}" for index in range(29)],
            self.joint_velocities,
        )
        _write_csv(directory / "body_pos.csv", ["body_0_x", "body_0_y", "body_0_z"], self.root_positions)
        _write_csv(
            directory / "body_quat.csv",
            ["body_0_w", "body_0_x", "body_0_y", "body_0_z"],
            self.root_quaternions_wxyz,
        )
        (directory / "metadata.txt").write_text(
            f"Metadata for: {directory.name}\n"
            "==============================\n\n"
            "Body part indexes:\n"
            "[0]\n\n"
            f"Total timesteps: {len(self.joint_positions)}\n",
            encoding="utf-8",
        )
        metadata = {
            "format": "sonic-g1-reference-v1",
            "source": self.source,
            "rate_hz": self.rate_hz,
            "frame_count": len(self.joint_positions),
            "duration_seconds": (len(self.joint_positions) - 1) / self.rate_hz,
            "joint_order": "IsaacLab",
            "joint_names": list(SONIC_G1_JOINT_ORDER),
            "source_frames": self.source_frames,
            "quaternion_order": "wxyz",
            "world_axes": "X-forward, Y-left, Z-up",
        }
        if self.retargeting_config is not None:
            metadata["retargeting_config"] = self.retargeting_config
        (directory / "trajectory_metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        (directory / "info.txt").write_text(
            f"Source: {self.source}\n"
            f"Frames: {len(self.joint_positions)}\n"
            f"Rate: {self.rate_hz} Hz\n"
            f"Duration: {(len(self.joint_positions) - 1) / self.rate_hz:.3f} s\n"
            "Joints: 29, IsaacLab order\n"
            "Root bodies: 1\n"
            "Quaternion order: wxyz\n",
            encoding="utf-8",
        )
        return directory


def _write_csv(path: Path, header: Sequence[str], rows: Sequence[Sequence[float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(header)
        writer.writerows(rows)

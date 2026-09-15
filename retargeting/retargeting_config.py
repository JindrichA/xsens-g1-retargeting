"""Shared, dependency-free configuration for recorded and live G1 retargeting."""

from __future__ import annotations

from dataclasses import dataclass


OUTPUT_RATE = 50
ROOT_HEIGHT = 0.76
NEAR_LIMIT_MARGIN = 0.05
TASK_SEGMENTS = {
    "torso_link": "T8",
    "left_wrist_yaw_link": "LeftHand",
    "right_wrist_yaw_link": "RightHand",
    "left_ankle_roll_link": "LeftFoot",
    "right_ankle_roll_link": "RightFoot",
}
TASK_LABELS = {
    "torso_link": "torso",
    "left_wrist_yaw_link": "left_wrist",
    "right_wrist_yaw_link": "right_wrist",
    "left_ankle_roll_link": "left_ankle",
    "right_ankle_roll_link": "right_ankle",
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


@dataclass(frozen=True)
class RetargetingConfig:
    scale: float = 0.75
    iterations: int = 4
    max_joint_speed: float = 4.0
    solver: str = "quadprog"
    output_rate: int = OUTPUT_RATE
    near_limit_margin: float = NEAR_LIMIT_MARGIN

    def __post_init__(self) -> None:
        if self.scale <= 0:
            raise ValueError("retargeting scale must be positive")
        if self.iterations <= 0:
            raise ValueError("IK iterations must be positive")
        if self.max_joint_speed <= 0:
            raise ValueError("maximum joint speed must be positive")
        if self.output_rate <= 0:
            raise ValueError("output rate must be positive")
        if self.near_limit_margin < 0:
            raise ValueError("near-limit margin cannot be negative")
        if not self.solver:
            raise ValueError("IK solver must be named")

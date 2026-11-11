"""Build and validate G1 EDU+ LowCmd messages without creating DDS transport."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

from .g1_dds_diagnostics import G1_HARDWARE_JOINT_ORDER, G1_RECORDED_FIRMWARE
from .sonic_stream import load_trajectory, select_trajectory_window
from .trajectory import SONIC_G1_JOINT_ORDER


SCHEMA = "g1-lowcmd-offline-dry-run-v1"
COMMAND_RATE_HZ = 500
SOURCE_RATE_HZ = 50
MODE_PR = 0
MODE_MACHINE = 5
BODY_MOTOR_COUNT = 29
MESSAGE_MOTOR_COUNT = 35
SDK_MASTER_COMMIT = "814556d15970dd2ecf1c9984e845ca02ab07e206"
SDK_LOW_LEVEL_EXAMPLE_URL = (
    "https://github.com/unitreerobotics/unitree_sdk2_python/blob/"
    f"{SDK_MASTER_COMMIT}/example/g1/low_level/g1_low_level_example.py"
)
SDK_EXAMPLE_KP = (
    60, 60, 60, 100, 40, 40,
    60, 60, 60, 100, 40, 40,
    60, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
)
SDK_EXAMPLE_KD = (
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
)
GAIN_PROFILES = {
    "zero": ((0.0,) * BODY_MOTOR_COUNT, (0.0,) * BODY_MOTOR_COUNT),
    "sdk-master-example": (SDK_EXAMPLE_KP, SDK_EXAMPLE_KD),
}

# Official mode-5 g1_29dof_rev_1_0 URDF limits. These are format-validation
# bounds, not manufacturer approval for physical commissioning.
POSITION_LIMITS = (
    (-2.5307, 2.8798), (-0.5236, 2.9671), (-2.7576, 2.7576), (-0.087267, 2.8798),
    (-0.87267, 0.5236), (-0.2618, 0.2618), (-2.5307, 2.8798), (-2.9671, 0.5236),
    (-2.7576, 2.7576), (-0.087267, 2.8798), (-0.87267, 0.5236), (-0.2618, 0.2618),
    (-2.618, 2.618), (-0.52, 0.52), (-0.52, 0.52), (-3.0892, 2.6704),
    (-1.5882, 2.2515), (-2.618, 2.618), (-1.0472, 2.0944),
    (-1.972222054, 1.972222054), (-1.614429558, 1.614429558),
    (-1.614429558, 1.614429558), (-3.0892, 2.6704), (-2.2515, 1.5882),
    (-2.618, 2.618), (-1.0472, 2.0944), (-1.972222054, 1.972222054),
    (-1.614429558, 1.614429558), (-1.614429558, 1.614429558),
)
VELOCITY_LIMITS = (
    32, 20, 32, 20, 30, 30, 32, 20, 32, 20, 30, 30, 32, 30, 30,
    37, 37, 37, 37, 37, 22, 22, 37, 37, 37, 37, 37, 22, 22,
)


def load_hardware_window(directory: Path, start_frame: int, frame_count: int):
    positions, velocities, quaternions = load_trajectory(directory)
    positions, velocities, _ = select_trajectory_window(
        positions, velocities, quaternions, start_frame, frame_count
    )
    sonic_indexes = {name.removesuffix("_joint"): index for index, name in enumerate(SONIC_G1_JOINT_ORDER)}
    indexes = [sonic_indexes[name] for name in G1_HARDWARE_JOINT_ORDER]
    return (
        [[row[index] for index in indexes] for row in positions],
        [[row[index] for index in indexes] for row in velocities],
    )


def validate_target(positions, velocities) -> None:
    if len(positions) != BODY_MOTOR_COUNT or len(velocities) != BODY_MOTOR_COUNT:
        raise ValueError("G1 EDU+ targets must contain exactly 29 body joints")
    for index, (position, velocity) in enumerate(zip(positions, velocities)):
        if not math.isfinite(position) or not math.isfinite(velocity):
            raise ValueError(f"non-finite target at joint {index}")
        lower, upper = POSITION_LIMITS[index]
        if not lower <= position <= upper:
            raise ValueError(
                f"{G1_HARDWARE_JOINT_ORDER[index]} position {position:.6f} exceeds "
                f"official URDF bounds [{lower:.6f}, {upper:.6f}]"
            )
        if abs(velocity) > VELOCITY_LIMITS[index]:
            raise ValueError(
                f"{G1_HARDWARE_JOINT_ORDER[index]} velocity {velocity:.6f} exceeds "
                f"official URDF magnitude {VELOCITY_LIMITS[index]:.6f}"
            )


def interpolate_targets(positions, velocities):
    ratio = COMMAND_RATE_HZ // SOURCE_RATE_HZ
    if COMMAND_RATE_HZ % SOURCE_RATE_HZ:
        raise RuntimeError("command rate must be an integer multiple of source rate")
    for frame in range(len(positions) - 1):
        for step in range(ratio):
            alpha = step / ratio
            yield (
                [
                    (1.0 - alpha) * positions[frame][joint] + alpha * positions[frame + 1][joint]
                    for joint in range(BODY_MOTOR_COUNT)
                ],
                [
                    (1.0 - alpha) * velocities[frame][joint] + alpha * velocities[frame + 1][joint]
                    for joint in range(BODY_MOTOR_COUNT)
                ],
            )
    yield positions[-1][:], velocities[-1][:]


def build_lowcmd(
    command_factory,
    crc,
    positions,
    velocities,
    *,
    enabled: bool,
    gain_profile: str = "zero",
):
    validate_target(positions, velocities)
    if gain_profile not in GAIN_PROFILES:
        raise ValueError(f"unknown gain profile {gain_profile!r}")
    kp, kd = GAIN_PROFILES[gain_profile]
    command = command_factory()
    if len(command.motor_cmd) != MESSAGE_MOTOR_COUNT:
        raise RuntimeError("Unitree HG LowCmd must contain 35 motor slots")
    command.mode_pr = MODE_PR
    command.mode_machine = MODE_MACHINE
    for index in range(BODY_MOTOR_COUNT):
        motor = command.motor_cmd[index]
        motor.mode = 1 if enabled else 0
        motor.q = positions[index]
        motor.dq = velocities[index] if enabled else 0.0
        motor.tau = 0.0
        motor.kp = kp[index] if enabled else 0.0
        motor.kd = kd[index] if enabled else 0.0
    for motor in command.motor_cmd[BODY_MOTOR_COUNT:]:
        motor.mode = 0
        motor.q = 0.0
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = 0.0
        motor.kd = 0.0
    command.crc = crc.Crc(command)
    if command.crc != crc.Crc(command):
        raise RuntimeError("LowCmd CRC verification failed")
    return command


def simulate_dry_run(
    positions,
    velocities,
    command_factory,
    crc,
    *,
    watchdog_ms: float,
    inject_stale_after_ms: float | None = None,
    gain_profile: str = "zero",
) -> dict:
    if watchdog_ms <= 0:
        raise ValueError("watchdog threshold must be positive")
    watchdog_ticks = max(1, math.ceil(watchdog_ms * COMMAND_RATE_HZ / 1000.0))
    stale_tick = (
        None
        if inject_stale_after_ms is None
        else max(0, math.ceil(inject_stale_after_ms * COMMAND_RATE_HZ / 1000.0))
    )
    command_count = 0
    enabled_count = 0
    disabled_count = 0
    last_fresh_tick = 0
    last_positions = positions[0]
    last_velocities = velocities[0]
    crc_first = None
    crc_last = None

    for tick, (target_positions, target_velocities) in enumerate(
        interpolate_targets(positions, velocities)
    ):
        fresh = stale_tick is None or tick < stale_tick
        if fresh:
            last_fresh_tick = tick
            last_positions = target_positions
            last_velocities = target_velocities
        enabled = fresh or tick - last_fresh_tick < watchdog_ticks
        command = build_lowcmd(
            command_factory,
            crc,
            last_positions,
            last_velocities,
            enabled=enabled,
            gain_profile=gain_profile,
        )
        crc_first = command.crc if crc_first is None else crc_first
        crc_last = command.crc
        command_count += 1
        enabled_count += int(enabled)
        disabled_count += int(not enabled)

    return {
        "schema": SCHEMA,
        "dry_run_only": True,
        "transmit_capability": False,
        "publisher_created": False,
        "firmware_recorded": G1_RECORDED_FIRMWARE,
        "robot_variant": "G1 EDU+ / dummy hands / mode_machine=5",
        "command_rate_hz": COMMAND_RATE_HZ,
        "source_rate_hz": SOURCE_RATE_HZ,
        "source_frame_count": len(positions),
        "command_count": command_count,
        "enabled_template_count": enabled_count,
        "disabled_template_count": disabled_count,
        "watchdog_ms": watchdog_ms,
        "injected_stale_after_ms": inject_stale_after_ms,
        "crc_first": crc_first,
        "crc_last": crc_last,
        "motor_slots": MESSAGE_MOTOR_COUNT,
        "body_slots_populated": BODY_MOTOR_COUNT,
        "unused_slots_disabled": True,
        "gain_profile": gain_profile,
        "gains": "zero" if gain_profile == "zero" else "official SDK example values; not commissioning approval",
        "gain_source_commit": SDK_MASTER_COMMIT if gain_profile == "sdk-master-example" else None,
        "gain_source_url": SDK_LOW_LEVEL_EXAMPLE_URL if gain_profile == "sdk-master-example" else None,
        "feedforward_torque": "zero",
        "bounds_source": "official g1_29dof_rev_1_0 URDF",
        "watchdog_action": "build disabled templates only; physical stop semantics remain unverified",
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--start-frame", type=int, required=True)
    parser.add_argument("--frame-count", type=int, required=True)
    parser.add_argument("--watchdog-ms", type=float, default=20.0)
    parser.add_argument("--inject-stale-after-ms", type=float)
    parser.add_argument(
        "--gain-profile",
        choices=tuple(GAIN_PROFILES),
        default="zero",
        help="offline-only gains; SDK example values are not physical commissioning approval",
    )
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "unitree_sdk2_python",
    )
    args = parser.parse_args(argv)
    if args.frame_count < 2:
        parser.error("--frame-count must be at least 2")
    if args.inject_stale_after_ms is not None and args.inject_stale_after_ms < 0:
        parser.error("--inject-stale-after-ms cannot be negative")

    sdk_root = args.sdk_root.expanduser().resolve()
    if not (sdk_root / "unitree_sdk2py/idl/default.py").is_file():
        parser.error(f"Unitree SDK2 Python source not found: {sdk_root}")
    sys.path.insert(0, str(sdk_root))
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    try:
        positions, velocities = load_hardware_window(
            args.trajectory, args.start_frame, args.frame_count
        )
        report = simulate_dry_run(
            positions,
            velocities,
            unitree_hg_msg_dds__LowCmd_,
            CRC(),
            watchdog_ms=args.watchdog_ms,
            inject_stale_after_ms=args.inject_stale_after_ms,
            gain_profile=args.gain_profile,
        )
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))
    report["trajectory"] = str(args.trajectory.expanduser().resolve())
    report["start_frame"] = args.start_frame
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

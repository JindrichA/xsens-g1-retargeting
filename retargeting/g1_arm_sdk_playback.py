"""Guarded G1 EDU+ arms-only playback through Unitree's high-level arm SDK."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

from .g1_dds_diagnostics import G1_HARDWARE_JOINT_ORDER, validate_interface
from .g1_lowcmd_dry_run import POSITION_LIMITS, SDK_MASTER_COMMIT, load_hardware_window


ARM_TOPIC = "rt/arm_sdk"
STATE_TOPIC = "rt/lowstate"
PROFILE = "g1-edu-plus-arm-sdk-v3"
EXECUTE_ACKNOWLEDGEMENT = "G1_ARM_AREA_CLEAR_REMOTE_ESTOP_READY"
ARM_INDICES = tuple(range(15, 29))
WAIST_INDICES = (12, 13, 14)
UPPER_BODY_INDICES = WAIST_INDICES + ARM_INDICES
WEIGHT_INDEX = 29
MESSAGE_MOTOR_COUNT = 35
RATE_HZ = 50
KP = 60.0
KD = 1.5
MAX_SOURCE_SECONDS = 26.0
MAX_MOTION_SCALE = 0.4
MAX_SPEED_SCALE = 0.1
MAX_DISPLACEMENT_RAD = 0.55
MAX_SPEED_RAD_S = 0.16
MIN_OBSERVED_MOTION_RAD = 0.01
MAX_TRACKING_ERROR_RAD = 0.25
MAX_TILT_RAD = 0.2
MAX_TEMPERATURE_C = 55.0
MAX_START_TEMPERATURE_C = 45.0
MAX_STATE_AGE_SECONDS = 0.1
ACTIVATE_SECONDS = 3.0
RETURN_SECONDS = 5.0
RELEASE_SECONDS = 3.0
HANDOFF_SETTLE_SECONDS = 1.0
SWITCH_TO_USER_CTRL_API_ID = 7110
SWITCH_TO_INTERNAL_CTRL_API_ID = 7111
REGULAR_MODE_FSM_IDS = (500, 501, 801, 802)


def prepare_arm_plan(
    positions,
    *,
    motion_scale: float = MAX_MOTION_SCALE,
    speed_scale: float = MAX_SPEED_SCALE,
):
    if len(positions) < 2:
        raise ValueError("arm playback requires at least two source frames")
    if not 0 < motion_scale <= MAX_MOTION_SCALE:
        raise ValueError(f"motion scale must be in (0, {MAX_MOTION_SCALE}]")
    if not 0 < speed_scale <= MAX_SPEED_SCALE:
        raise ValueError(f"speed scale must be in (0, {MAX_SPEED_SCALE}]")
    if any(len(frame) != len(G1_HARDWARE_JOINT_ORDER) for frame in positions):
        raise ValueError("source frames must contain exactly 29 G1 joints")

    first = positions[0]
    deltas = []
    for frame in positions:
        row = [motion_scale * (frame[index] - first[index]) for index in ARM_INDICES]
        if not all(math.isfinite(value) for value in row):
            raise ValueError("arm plan contains non-finite values")
        deltas.append(row)

    playback_seconds = (len(deltas) - 1) / (RATE_HZ * speed_scale)
    maximum_displacement = max(abs(value) for row in deltas for value in row)
    if maximum_displacement > MAX_DISPLACEMENT_RAD:
        raise ValueError(
            f"scaled arm displacement {maximum_displacement:.3f} rad exceeds "
            f"{MAX_DISPLACEMENT_RAD:.3f} rad"
        )
    maximum_speed = max(
        abs(deltas[frame][joint] - deltas[frame - 1][joint]) * RATE_HZ * speed_scale
        for frame in range(1, len(deltas))
        for joint in range(len(ARM_INDICES))
    )
    if maximum_speed > MAX_SPEED_RAD_S:
        raise ValueError(
            f"scaled arm speed {maximum_speed:.3f} rad/s exceeds {MAX_SPEED_RAD_S:.3f} rad/s"
        )
    return deltas, {
        "source_frames": len(deltas),
        "source_seconds": (len(deltas) - 1) / RATE_HZ,
        "playback_seconds": playback_seconds,
        "motion_scale": motion_scale,
        "speed_scale": speed_scale,
        "maximum_displacement_rad": maximum_displacement,
        "maximum_speed_rad_s": maximum_speed,
    }


def target_from_delta(initial_positions, delta):
    if len(initial_positions) != len(UPPER_BODY_INDICES) or len(delta) != len(ARM_INDICES):
        raise ValueError("targets must hold 3 waist joints and apply deltas to 14 arm joints")
    target = list(initial_positions[: len(WAIST_INDICES)]) + [
        initial_positions[len(WAIST_INDICES) + joint] + delta[joint]
        for joint in range(len(ARM_INDICES))
    ]
    for offset, value in enumerate(target):
        hardware_index = UPPER_BODY_INDICES[offset]
        lower, upper = POSITION_LIMITS[hardware_index]
        if not lower <= value <= upper:
            name = G1_HARDWARE_JOINT_ORDER[hardware_index]
            raise RuntimeError(f"{name} target {value:.6f} exceeds [{lower:.6f}, {upper:.6f}]")
    return target


def interpolate_delta(deltas, elapsed_seconds: float, speed_scale: float):
    source_position = max(0.0, elapsed_seconds * RATE_HZ * speed_scale)
    lower = min(int(source_position), len(deltas) - 1)
    upper = min(lower + 1, len(deltas) - 1)
    alpha = source_position - lower
    return [
        (1.0 - alpha) * deltas[lower][joint] + alpha * deltas[upper][joint]
        for joint in range(len(ARM_INDICES))
    ]


def build_arm_command(command_factory, crc, targets, *, weight: float, mode_machine: int = 5):
    if len(targets) != len(UPPER_BODY_INDICES):
        raise ValueError("arm SDK command requires 3 waist-hold and 14 arm targets")
    if not 0.0 <= weight <= 1.0:
        raise ValueError("arm SDK weight must be in [0, 1]")
    command = command_factory()
    if len(command.motor_cmd) != MESSAGE_MOTOR_COUNT:
        raise RuntimeError("Unitree HG LowCmd must contain 35 motor slots")
    command.mode_pr = 0
    command.mode_machine = mode_machine
    for offset, hardware_index in enumerate(UPPER_BODY_INDICES):
        motor = command.motor_cmd[hardware_index]
        motor.mode = 1
        motor.q = targets[offset]
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = KP
        motor.kd = KD
    command.motor_cmd[WEIGHT_INDEX].q = weight
    command.crc = crc.Crc(command)
    if command.crc != crc.Crc(command):
        raise RuntimeError("arm SDK command CRC verification failed")
    return command


def validate_state(state, calculated_crc: int, *, captured_at: float, now: float) -> None:
    if now - captured_at > MAX_STATE_AGE_SECONDS:
        raise RuntimeError("LowState is stale")
    if int(state.crc) != int(calculated_crc):
        raise RuntimeError("LowState CRC is invalid")
    if int(state.mode_machine) != 5 or int(state.mode_pr) != 0:
        raise RuntimeError(
            f"expected mode_machine=5 and mode_pr=0, got {state.mode_machine} and {state.mode_pr}"
        )
    if max(abs(float(value)) for value in state.imu_state.rpy[:2]) > MAX_TILT_RAD:
        raise RuntimeError("robot tilt exceeds the arms-only playback limit")
    for index in UPPER_BODY_INDICES:
        motor = state.motor_state[index]
        values = [float(motor.q), float(motor.dq), float(motor.tau_est)]
        temperatures = [float(value) for value in motor.temperature]
        if not all(math.isfinite(value) for value in (*values, *temperatures)):
            raise RuntimeError(f"non-finite arm state at joint {index}")
        if max(temperatures) > MAX_TEMPERATURE_C:
            raise RuntimeError(f"arm temperature exceeds {MAX_TEMPERATURE_C:.0f} C")
        if int(motor.motorstate) != 0:
            raise RuntimeError(f"motor error at arm joint {index}: {motor.motorstate}")


def execute_plan(interface: str, deltas, speed_scale: float, sdk_root: Path) -> dict:
    sys.path.insert(0, str(sdk_root))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.utils.crc import CRC

    crc = CRC()
    lock = threading.Lock()
    latest = {"state": None, "calculated_crc": None, "captured_at": 0.0}
    interrupted = False

    def state_handler(state):
        calculated_crc = crc.Crc(state)
        with lock:
            latest.update(
                state=state,
                calculated_crc=calculated_crc,
                captured_at=time.monotonic(),
            )

    def snapshot():
        with lock:
            return latest.copy()

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    ChannelFactoryInitialize(0, interface)
    subscriber = ChannelSubscriber(STATE_TOPIC, LowState_)
    subscriber.Init(state_handler, 10)

    deadline = time.monotonic() + 5.0
    observed_ticks = []
    while len(observed_ticks) < 3 and time.monotonic() < deadline:
        current = snapshot()
        if current["state"] is not None:
            validate_state(
                current["state"],
                current["calculated_crc"],
                captured_at=current["captured_at"],
                now=time.monotonic(),
            )
            tick = int(current["state"].tick)
            if not observed_ticks or tick != observed_ticks[-1]:
                observed_ticks.append(tick)
        time.sleep(0.01)
    if len(observed_ticks) < 3 or any(b <= a for a, b in zip(observed_ticks, observed_ticks[1:])):
        raise RuntimeError("three advancing, CRC-valid LowState samples were not observed")

    state = snapshot()["state"]
    start_temperature = max(
        float(value)
        for index in UPPER_BODY_INDICES
        for value in state.motor_state[index].temperature
    )
    if start_temperature > MAX_START_TEMPERATURE_C:
        raise RuntimeError(
            f"upper-body temperature {start_temperature:.0f} C exceeds the "
            f"{MAX_START_TEMPERATURE_C:.0f} C cool-start limit"
        )
    initial = [float(state.motor_state[index].q) for index in UPPER_BODY_INDICES]
    publisher = ChannelPublisher(ARM_TOPIC, LowCmd_)
    publisher.Init()
    period = 1.0 / RATE_HZ
    last_targets = initial
    maximum_observed_displacement = 0.0
    maximum_tracking_error = 0.0

    def publish(target, weight):
        current = snapshot()["state"]
        command = build_arm_command(
            unitree_hg_msg_dds__LowCmd_,
            crc,
            target,
            weight=weight,
            mode_machine=int(current.mode_machine),
        )
        publisher.Write(command)

    def checked_state(expected=None):
        nonlocal maximum_observed_displacement, maximum_tracking_error
        current = snapshot()
        validate_state(
            current["state"],
            current["calculated_crc"],
            captured_at=current["captured_at"],
            now=time.monotonic(),
        )
        observed = [float(current["state"].motor_state[index].q) for index in ARM_INDICES]
        maximum_observed_displacement = max(
            maximum_observed_displacement,
            *(
                abs(observed[offset] - initial[len(WAIST_INDICES) + offset])
                for offset in range(len(ARM_INDICES))
            ),
        )
        if expected is not None:
            observed = [
                float(current["state"].motor_state[index].q) for index in UPPER_BODY_INDICES
            ]
            error = max(
                abs(observed[offset] - expected[offset])
                for offset in range(len(UPPER_BODY_INDICES))
            )
            maximum_tracking_error = max(maximum_tracking_error, error)
            if error > MAX_TRACKING_ERROR_RAD:
                raise RuntimeError(f"arm tracking error {error:.3f} rad exceeds limit")

    def run_stage(duration, target_at, weight_at, *, enforce_tracking=True):
        nonlocal last_targets
        start = time.monotonic()
        while not interrupted:
            elapsed = time.monotonic() - start
            ratio = min(1.0, elapsed / duration) if duration else 1.0
            target = target_at(ratio)
            checked_state(last_targets if enforce_tracking else None)
            publish(target, weight_at(ratio))
            last_targets = target
            if ratio >= 1.0:
                return
            time.sleep(max(0.0, period - (time.monotonic() - start - elapsed)))
        raise RuntimeError("arm playback interrupted")

    def run_playback():
        nonlocal maximum_observed_displacement, maximum_tracking_error
        try:
            run_stage(
                ACTIVATE_SECONDS,
                lambda _ratio: initial,
                lambda ratio: ratio,
                enforce_tracking=False,
            )
            maximum_observed_displacement = 0.0
            maximum_tracking_error = 0.0
            playback_seconds = (len(deltas) - 1) / (RATE_HZ * speed_scale)
            start = time.monotonic()
            run_stage(
                playback_seconds,
                lambda _ratio: target_from_delta(
                    initial,
                    interpolate_delta(deltas, time.monotonic() - start, speed_scale),
                ),
                lambda _ratio: 1.0,
            )
            commanded_displacement = max(abs(value) for row in deltas for value in row)
            if (
                commanded_displacement >= MIN_OBSERVED_MOTION_RAD
                and maximum_observed_displacement < MIN_OBSERVED_MOTION_RAD
            ):
                raise RuntimeError(
                    "arm SDK command was not measurably tracked: "
                    f"commanded={commanded_displacement:.4f} rad, "
                    f"observed={maximum_observed_displacement:.4f} rad"
                )
            return_start = last_targets[:]
            run_stage(
                RETURN_SECONDS,
                lambda ratio: [
                    (1.0 - ratio) * return_start[joint] + ratio * initial[joint]
                    for joint in range(len(UPPER_BODY_INDICES))
                ],
                lambda _ratio: 1.0,
            )
        finally:
            release_start = last_targets[:]
            start = time.monotonic()
            while time.monotonic() - start < RELEASE_SECONDS:
                ratio = min(1.0, (time.monotonic() - start) / RELEASE_SECONDS)
                publish(release_start, 1.0 - ratio)
                time.sleep(period)
            publish(release_start, 0.0)

    loco = LocoClient()
    loco.SetTimeout(5.0)
    loco.Init()
    loco._RegistApi(SWITCH_TO_USER_CTRL_API_ID, 0)
    loco._RegistApi(SWITCH_TO_INTERNAL_CTRL_API_ID, 0)
    fsm_code, fsm_id = loco.GetFsmId()
    regular_mode_active = fsm_code == 0 and fsm_id in REGULAR_MODE_FSM_IDS
    user_control_handoff = False
    if not regular_mode_active:
        switch_code, _ = loco._Call(SWITCH_TO_USER_CTRL_API_ID, json.dumps({"data": False}))
        if switch_code != 0:
            raise RuntimeError(f"SwitchToUserCtrl failed with code {switch_code}")
        user_control_handoff = True
    restore_code = None
    try:
        if user_control_handoff:
            time.sleep(HANDOFF_SETTLE_SECONDS)
            current = snapshot()
            validate_state(
                current["state"],
                current["calculated_crc"],
                captured_at=current["captured_at"],
                now=time.monotonic(),
            )
            initial = [float(current["state"].motor_state[index].q) for index in UPPER_BODY_INDICES]
            for delta in deltas:
                target_from_delta(initial, delta)
            last_targets = initial
            maximum_observed_displacement = 0.0
            maximum_tracking_error = 0.0
        run_playback()
    finally:
        if user_control_handoff:
            restore_code, _ = loco._Call(
                SWITCH_TO_INTERNAL_CTRL_API_ID,
                json.dumps({"data": 0}),
            )
    if user_control_handoff and restore_code != 0:
        raise RuntimeError(f"SwitchToInternalCtrl(LAST) failed with code {restore_code}")

    return {
        "executed": True,
        "arm_topic": ARM_TOPIC,
        "locomotion_service_released": False,
        "initial_locomotion_fsm": fsm_id,
        "regular_mode_preserved": regular_mode_active,
        "user_control_handoff": user_control_handoff,
        "handoff_settle_seconds": HANDOFF_SETTLE_SECONDS if user_control_handoff else 0.0,
        "internal_control_restored": not user_control_handoff or restore_code == 0,
        "final_arm_sdk_weight": 0.0,
        "maximum_observed_displacement_rad": maximum_observed_displacement,
        "maximum_tracking_error_rad": maximum_tracking_error,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--start-frame", type=int, required=True)
    parser.add_argument("--frame-count", type=int, required=True)
    parser.add_argument("--motion-scale", type=float, default=MAX_MOTION_SCALE)
    parser.add_argument("--speed-scale", type=float, default=MAX_SPEED_SCALE)
    parser.add_argument("--profile", choices=(PROFILE,))
    parser.add_argument("--interface")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--acknowledge", default="")
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "unitree_sdk2_python",
    )
    args = parser.parse_args(argv)
    if args.frame_count < 2 or (args.frame_count - 1) / RATE_HZ > MAX_SOURCE_SECONDS:
        parser.error(f"source window must contain 2 to {int(MAX_SOURCE_SECONDS * RATE_HZ) + 1} frames")

    try:
        positions, _ = load_hardware_window(args.trajectory, args.start_frame, args.frame_count)
        deltas, summary = prepare_arm_plan(
            positions,
            motion_scale=args.motion_scale,
            speed_scale=args.speed_scale,
        )
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))

    report = {
        "schema": "g1-arm-sdk-playback-v1",
        "mode": "execute" if args.execute else "dry-run",
        "profile": args.profile or PROFILE,
        "sdk_reference_commit": SDK_MASTER_COMMIT,
        "arm_topic": ARM_TOPIC,
        "low_level_topic_used": False,
        "motion_service_released": False,
        "joints": [G1_HARDWARE_JOINT_ORDER[index] for index in ARM_INDICES],
        "held_joints": [G1_HARDWARE_JOINT_ORDER[index] for index in WAIST_INDICES],
        "gains_source": "official g1_arm7_sdk_dds_example.py",
        "kp": KP,
        "kd": KD,
        "activation_seconds": ACTIVATE_SECONDS,
        "return_seconds": RETURN_SECONDS,
        "release_seconds": RELEASE_SECONDS,
        **summary,
    }
    if not args.execute:
        print(json.dumps(report, indent=2))
        return 0
    if args.profile != PROFILE:
        parser.error(f"--execute requires --profile {PROFILE}")
    if not args.interface:
        parser.error("--execute requires --interface")
    if args.acknowledge != EXECUTE_ACKNOWLEDGEMENT:
        parser.error(f"--execute requires --acknowledge {EXECUTE_ACKNOWLEDGEMENT}")
    try:
        interface = validate_interface(args.interface)
        sdk_root = args.sdk_root.expanduser().resolve()
        if not (sdk_root / "unitree_sdk2py/core/channel.py").is_file():
            raise RuntimeError(f"Unitree SDK2 Python source not found: {sdk_root}")
        report.update(execute_plan(interface, deltas, args.speed_scale, sdk_root))
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

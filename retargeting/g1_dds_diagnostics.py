"""Subscriber-only Unitree G1 low-state diagnostic; never creates a writer."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time


G1_HARDWARE_JOINT_ORDER = (
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
)

G1_RECORDED_FIRMWARE = "V1.5.1.1"
G1_RECORDED_VARIANT = "G1 EDU+"
G1_RECORDED_HANDS = "dummy hands"
DEFAULT_SAMPLE_COUNT = 3


def validate_interface(value: str) -> str:
    interface = value.strip()
    if not interface or "/" in interface or interface in {"lo", "loopback"}:
        raise ValueError("physical G1 diagnostics require an explicit non-loopback interface")
    if not interface.startswith(("en", "eth")):
        raise ValueError("physical G1 diagnostics require a dedicated wired Ethernet interface")
    if not (Path("/sys/class/net") / interface).exists():
        raise ValueError(f"network interface does not exist: {interface}")
    return interface


def summarize_low_state(
    state,
    calculated_crc: int,
    *,
    recorded_firmware: str = G1_RECORDED_FIRMWARE,
) -> dict:
    joints = []
    for index, name in enumerate(G1_HARDWARE_JOINT_ORDER):
        motor = state.motor_state[index]
        values = [float(motor.q), float(motor.dq), float(motor.tau_est), float(motor.vol)]
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"non-finite motor state at index {index}")
        joints.append(
            {
                "index": index,
                "name": name,
                "q": values[0],
                "dq": values[1],
                "tau_est": values[2],
                "temperature": list(motor.temperature),
                "voltage": values[3],
                "motorstate": int(motor.motorstate),
            }
        )
    return {
        "schema": "g1-read-only-lowstate-v1",
        "read_only": True,
        "transmit_capability": False,
        "topic": "rt/lowstate",
        "dds_domain": 0,
        "version_raw": list(state.version),
        "firmware_recorded": recorded_firmware,
        "firmware_verification": "OPERATOR_RECORDED_NOT_DDS_VERIFIED",
        "tick": int(state.tick),
        "mode_pr": int(state.mode_pr),
        "mode_machine": int(state.mode_machine),
        "crc_received": int(state.crc),
        "crc_calculated": int(calculated_crc),
        "crc_ok": int(state.crc) == int(calculated_crc),
        "imu_quaternion_wxyz": list(state.imu_state.quaternion),
        "imu_rpy": list(state.imu_state.rpy),
        "joints": joints,
        "robot_variant": G1_RECORDED_VARIANT,
        "robot_hands": G1_RECORDED_HANDS,
        "variant_verification": "OPERATOR_RECORDED_AND_29_BODY_SLOTS_OBSERVED",
    }


def summarize_low_states(
    states,
    calculated_crcs,
    *,
    recorded_firmware: str = G1_RECORDED_FIRMWARE,
) -> dict:
    if len(states) < 2:
        raise ValueError("at least two low-state samples are required")
    if len(states) != len(calculated_crcs):
        raise ValueError("state and CRC counts differ")

    versions = [tuple(int(value) for value in state.version) for state in states]
    ticks = [int(state.tick) for state in states]
    received_crcs = [int(state.crc) for state in states]
    calculated_crcs = [int(value) for value in calculated_crcs]
    report = summarize_low_state(
        states[-1],
        calculated_crcs[-1],
        recorded_firmware=recorded_firmware,
    )
    report["observation"] = {
        "sample_count": len(states),
        "ticks": ticks,
        "tick_advancing": any(current != previous for previous, current in zip(ticks, ticks[1:])),
        "version_stable": len(set(versions)) == 1,
        "mode_pr_stable": len({int(state.mode_pr) for state in states}) == 1,
        "mode_machine_stable": len({int(state.mode_machine) for state in states}) == 1,
        "crc_all_ok": all(
            received == calculated
            for received, calculated in zip(received_crcs, calculated_crcs)
        ),
    }
    observation = report["observation"]
    report["diagnostic_status"] = (
        "PASS"
        if observation["tick_advancing"]
        and observation["version_stable"]
        and observation["mode_pr_stable"]
        and observation["mode_machine_stable"]
        and observation["crc_all_ok"]
        else "FAIL"
    )
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLE_COUNT)
    parser.add_argument("--recorded-firmware", default=G1_RECORDED_FIRMWARE)
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "unitree_sdk2_python",
    )
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.samples < 2:
        parser.error("--samples must be at least 2 so state tick progression can be checked")
    if args.recorded_firmware != G1_RECORDED_FIRMWARE:
        parser.error(
            f"no reviewed read-only profile for firmware {args.recorded_firmware!r}; "
            f"expected {G1_RECORDED_FIRMWARE}"
        )
    try:
        interface = validate_interface(args.interface)
    except ValueError as error:
        parser.error(str(error))
    sdk_root = args.sdk_root.expanduser().resolve()
    if not (sdk_root / "unitree_sdk2py/core/channel.py").is_file():
        parser.error(f"Unitree SDK2 Python source not found: {sdk_root}")
    sys.path.insert(0, str(sdk_root))

    # Import only the subscriber path. This module contains no LowCmd or
    # ChannelPublisher import and cannot create an application-topic writer.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC

    ChannelFactoryInitialize(0, interface)
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init()
    try:
        deadline = time.monotonic() + args.timeout
        states = []
        while len(states) < args.samples:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            state = subscriber.Read(timeout=remaining)
            if state is None:
                break
            states.append(state)
        if len(states) < args.samples:
            raise RuntimeError(
                f"received {len(states)}/{args.samples} rt/lowstate samples within "
                f"{args.timeout:g} seconds"
            )
        crc = CRC()
        report = summarize_low_states(
            states,
            [crc.Crc(state) for state in states],
            recorded_firmware=args.recorded_firmware,
        )
        report["interface"] = interface
        print(json.dumps(report, indent=2))
    finally:
        subscriber.Close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

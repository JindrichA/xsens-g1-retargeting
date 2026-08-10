"""Publish exported G1 trajectories using SONIC's packed ZMQ protocol v1."""

from __future__ import annotations

import argparse
from array import array
import csv
import json
import math
from pathlib import Path
import sys
import time
from typing import Sequence

from .trajectory import SONIC_G1_JOINT_ORDER


HEADER_SIZE = 1280
TOPIC = b"pose"


class SonicStreamError(ValueError):
    """Raised when a trajectory cannot be safely streamed to SONIC."""


def load_trajectory(directory: str | Path) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
    directory = Path(directory).expanduser().resolve()
    metadata_path = directory / "trajectory_metadata.json"
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise SonicStreamError(f"missing trajectory metadata: {metadata_path}") from error
    except json.JSONDecodeError as error:
        raise SonicStreamError(f"invalid trajectory metadata: {error}") from error

    if metadata.get("format") != "sonic-g1-reference-v1":
        raise SonicStreamError("unsupported trajectory format")
    if metadata.get("rate_hz") != 50:
        raise SonicStreamError("SONIC trajectory rate must be 50 Hz")
    if metadata.get("joint_names") != list(SONIC_G1_JOINT_ORDER):
        raise SonicStreamError("trajectory joint names are not in SONIC IsaacLab order")
    if metadata.get("quaternion_order") != "wxyz":
        raise SonicStreamError("trajectory root quaternions must use wxyz order")

    joint_pos = _read_float_csv(directory / "joint_pos.csv", 29)
    joint_vel = _read_float_csv(directory / "joint_vel.csv", 29)
    body_quat = _read_float_csv(directory / "body_quat.csv", 4)
    frame_count = metadata.get("frame_count")
    if not isinstance(frame_count, int) or frame_count <= 0:
        raise SonicStreamError("trajectory frame_count must be a positive integer")
    if len(joint_pos) != frame_count or len(joint_vel) != frame_count or len(body_quat) != frame_count:
        raise SonicStreamError("trajectory CSV row counts do not match frame_count")
    for index, quaternion in enumerate(body_quat):
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not 0.99 <= norm <= 1.01:
            raise SonicStreamError(f"body quaternion at frame {index} is not normalized")
    return joint_pos, joint_vel, body_quat


def pack_protocol_v1(
    joint_pos: Sequence[Sequence[float]],
    joint_vel: Sequence[Sequence[float]],
    body_quat: Sequence[Sequence[float]],
    *,
    frame_indices: Sequence[int] | None = None,
    catch_up: bool = False,
) -> bytes:
    frame_count = len(joint_pos)
    if frame_count == 0:
        raise SonicStreamError("cannot stream an empty trajectory")
    if len(joint_vel) != frame_count or len(body_quat) != frame_count:
        raise SonicStreamError("SONIC fields must have the same frame count")
    _validate_shape_and_values("joint_pos", joint_pos, 29)
    _validate_shape_and_values("joint_vel", joint_vel, 29)
    _validate_shape_and_values("body_quat_w", body_quat, 4)
    for index, quaternion in enumerate(body_quat):
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not 0.99 <= norm <= 1.01:
            raise SonicStreamError(f"body_quat_w frame {index} is not normalized")
    if frame_indices is None:
        indexes = list(range(frame_count))
    else:
        if len(frame_indices) != frame_count:
            raise SonicStreamError("frame_index must have the same frame count as motion fields")
        if any(isinstance(value, bool) or not isinstance(value, int) for value in frame_indices):
            raise SonicStreamError("frame_index values must be integers")
        indexes = list(frame_indices)
        if any(current <= previous for previous, current in zip(indexes, indexes[1:])):
            raise SonicStreamError("frame_index values must be strictly increasing")

    header = {
        "v": 1,
        "endian": "le",
        "count": frame_count,
        "fields": [
            {"name": "joint_pos", "dtype": "f32", "shape": [frame_count, 29]},
            {"name": "joint_vel", "dtype": "f32", "shape": [frame_count, 29]},
            {"name": "body_quat_w", "dtype": "f32", "shape": [frame_count, 4]},
            {"name": "frame_index", "dtype": "i64", "shape": [frame_count]},
            {"name": "catch_up", "dtype": "u8", "shape": [1]},
        ],
    }
    header_bytes = json.dumps(header, separators=(",", ":")).encode("ascii")
    if len(header_bytes) > HEADER_SIZE:
        raise SonicStreamError("SONIC message header exceeds 1280 bytes")

    payload = bytearray(TOPIC)
    payload.extend(header_bytes.ljust(HEADER_SIZE, b"\0"))
    payload.extend(_float32_bytes(joint_pos))
    payload.extend(_float32_bytes(joint_vel))
    payload.extend(_float32_bytes(body_quat))
    payload.extend(_int64_bytes(indexes))
    payload.append(1 if catch_up else 0)
    return bytes(payload)


def publish(message: bytes, bind: str, startup_delay: float, hold_seconds: float) -> None:
    try:
        import zmq
    except ImportError as error:
        raise SonicStreamError("pyzmq is required; install it with: python3 -m pip install pyzmq") from error

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    try:
        socket.bind(bind)
        time.sleep(startup_delay)
        socket.send(message)
        time.sleep(hold_seconds)
    except zmq.ZMQError as error:
        raise SonicStreamError(f"ZMQ publish failed at {bind}: {error}") from error
    finally:
        socket.close()
        context.term()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Stream a 50 Hz G1 reference to SONIC in simulation")
    parser.add_argument("trajectory", type=Path, help="SONIC reference directory produced by batch_export.py")
    parser.add_argument(
        "--bind",
        default="tcp://127.0.0.1:5556",
        help="ZMQ PUB endpoint (default: loopback-only tcp://127.0.0.1:5556)",
    )
    parser.add_argument("--startup-delay", type=float, default=1.0, help="seconds for subscribers to connect")
    parser.add_argument("--hold-seconds", type=float, default=0.5, help="seconds to keep the socket open after send")
    parser.add_argument("--catch-up", action="store_true", help="allow SONIC to catch up after large frame gaps")
    parser.add_argument("--dry-run", action="store_true", help="validate and pack without opening a socket")
    args = parser.parse_args(argv)
    if args.startup_delay < 0 or args.hold_seconds < 0:
        parser.error("delays must be non-negative")

    try:
        joint_pos, joint_vel, body_quat = load_trajectory(args.trajectory)
        message = pack_protocol_v1(joint_pos, joint_vel, body_quat, catch_up=args.catch_up)
        print(f"Validated {len(joint_pos)} frames; packed {len(message)} bytes of SONIC protocol v1")
        if not args.dry_run:
            publish(message, args.bind, args.startup_delay, args.hold_seconds)
            print(f"Published trajectory on {args.bind}")
    except SonicStreamError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


def _read_float_csv(path: Path, width: int) -> list[list[float]]:
    try:
        with path.open(newline="", encoding="utf-8") as source:
            reader = csv.reader(source)
            next(reader)
            rows = [[float(value) for value in row] for row in reader]
    except FileNotFoundError as error:
        raise SonicStreamError(f"missing trajectory file: {path}") from error
    except (StopIteration, ValueError) as error:
        raise SonicStreamError(f"invalid numeric CSV: {path}") from error
    _validate_shape_and_values(path.name, rows, width)
    return rows


def _validate_shape_and_values(name: str, rows: Sequence[Sequence[float]], width: int) -> None:
    for index, row in enumerate(rows):
        if len(row) != width:
            raise SonicStreamError(f"{name} frame {index} has {len(row)} values; expected {width}")
        if not all(math.isfinite(value) for value in row):
            raise SonicStreamError(f"{name} frame {index} contains a non-finite value")


def _float32_bytes(rows: Sequence[Sequence[float]]) -> bytes:
    values = array("f", (value for row in rows for value in row))
    if sys.byteorder != "little":
        values.byteswap()
    return values.tobytes()


def _int64_bytes(values: Sequence[int]) -> bytes:
    packed = array("q", values)
    if packed.itemsize != 8:
        raise SonicStreamError("platform does not provide 64-bit signed arrays")
    if sys.byteorder != "little":
        packed.byteswap()
    return packed.tobytes()


if __name__ == "__main__":
    raise SystemExit(main())

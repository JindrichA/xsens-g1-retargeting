"""Bridge freshness-aware ROS 2 Xsens frames to an atomic canonical snapshot."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import sys
import time

from .live_frame import (
    CANONICAL_SEGMENTS,
    LiveFrame,
    LiveFrameError,
    LiveSegment,
    counter_advanced,
    encode_live_frame,
    encode_live_frame_json,
)


FRAME_NAMES = {
    re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower(): name for name in CANONICAL_SEGMENTS
}
FRAME_NAMES["right_forearm"] = "RightForeArm"
FRAME_NAMES["left_forearm"] = "LeftForeArm"


def quaternion_multiply(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def undo_driver_arm_rotation(name: str, quaternion):
    if name.startswith("Right") and name in {"RightUpperArm", "RightForeArm", "RightHand"}:
        return quaternion_multiply(quaternion, (0.7071068, 0.7071068, 0.0, 0.0))
    if name.startswith("Left") and name in {"LeftUpperArm", "LeftForeArm", "LeftHand"}:
        return quaternion_multiply(quaternion, (0.7071068, -0.7071068, 0.0, 0.0))
    return quaternion


def canonical_name(frame_id: str) -> str | None:
    name = frame_id.removeprefix("skeleton_")
    return FRAME_NAMES.get(name)


def frame_from_ros_message(message, receive_ns: int | None = None) -> LiveFrame:
    receive_ns = time.monotonic_ns() if receive_ns is None else receive_ns
    segments = {}
    for state in message.states:
        name = canonical_name(state.header.frame_id)
        if name is None:
            continue
        position = state.pose.position
        orientation = state.pose.orientation
        quaternion = (orientation.w, orientation.x, orientation.y, orientation.z)
        segments[name] = LiveSegment(
            position=(position.x, position.y, position.z),
            quaternion_wxyz=undo_driver_arm_rotation(name, quaternion),
        )
    frame = LiveFrame(
        source_sample_counter=int(message.source_sample_counter),
        source_frame_time_ms=int(message.source_frame_time_ms),
        source_receive_steady_ns=int(message.source_receive_steady_ns),
        capture_receive_steady_ns=receive_ns,
        segments=segments,
    )
    # Validation catches incomplete skeletons, NaNs, and malformed quaternions.
    encode_live_frame(frame)
    return frame


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/xsens-live-frame.json"))
    parser.add_argument("--topic", default="/link_states")
    args = parser.parse_args(argv)
    try:
        import rclpy
        from xsens_mvn_msgs.msg import LinkStateArray
    except ImportError as error:
        print(f"error: ROS 2 Jazzy and the extended xsens_mvn_msgs are required: {error}", file=sys.stderr)
        return 2
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    last_counter = None
    rclpy.init()
    node = rclpy.create_node("xsens_live_capture")

    def callback(message):
        nonlocal last_counter
        try:
            frame = frame_from_ros_message(message)
            if not counter_advanced(last_counter, frame.source_sample_counter):
                return
            last_counter = frame.source_sample_counter
            temporary = output.with_suffix(output.suffix + ".tmp")
            temporary.write_bytes(encode_live_frame_json(frame))
            os.replace(temporary, output)
        except LiveFrameError as error:
            node.get_logger().warning(str(error))

    node.create_subscription(LinkStateArray, args.topic, callback, 10)
    node.get_logger().info(f"Bridging {args.topic} to atomic snapshot {output}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

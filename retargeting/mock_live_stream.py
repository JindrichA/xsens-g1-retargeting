"""Replay an MVNX recording through the live Xsens atomic-file boundary."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

from .live_frame import CANONICAL_SEGMENTS, LiveFrame, LiveSegment, encode_live_frame_json
from .mvnx import MvnxError, MvnxFrame, iter_frames, read_metadata


def resampled_frames(path: Path, rate_hz: float):
    metadata = read_metadata(path)
    missing = sorted(set(CANONICAL_SEGMENTS) - set(metadata.segment_names))
    if missing:
        raise MvnxError(f"MVNX is missing canonical segments: {', '.join(missing)}")
    output_index = 0
    target_source_index = 0
    for frame in iter_frames(path, metadata):
        if frame.index < target_source_index:
            continue
        yield metadata, frame
        output_index += 1
        target_source_index = round(output_index * metadata.frame_rate / rate_hz)


def live_frame_from_mvnx(
    metadata, frame: MvnxFrame, counter: int, now_ns: int, rate_hz: float = 50.0
) -> LiveFrame:
    segment_index = {name: index for index, name in enumerate(metadata.segment_names)}
    return LiveFrame(
        source_sample_counter=counter & 0xFFFFFFFF,
        source_frame_time_ms=round(counter * 1000 / rate_hz),
        source_receive_steady_ns=now_ns,
        capture_receive_steady_ns=now_ns,
        segments={
            name: LiveSegment(
                position=frame.segment_positions[segment_index[name]],
                quaternion_wxyz=frame.segment_quaternions[segment_index[name]],
            )
            for name in CANONICAL_SEGMENTS
        },
    )


def control_enabled(path: Path) -> bool:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return isinstance(value, dict) and value.get("enabled") is True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mvnx", type=Path, help="recording to replay")
    parser.add_argument("--output", type=Path, default=Path("/tmp/xsens-live-frame.json"))
    parser.add_argument("--rate", type=float, default=50.0, help="output frames per second")
    parser.add_argument("--loop", action="store_true", help="restart the recording at its end")
    parser.add_argument("--control-gated", action="store_true", help="hold frame zero until live control is enabled")
    parser.add_argument("--control", type=Path, default=Path("/tmp/xsens-live-control.json"))
    parser.add_argument("--start-hold", type=float, default=3.0, help="frame-zero hold after enable")
    args = parser.parse_args(argv)
    if args.rate <= 0 or args.start_hold < 0:
        parser.error("rate must be positive and start-hold must be non-negative")
    source = args.mvnx.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not source.is_file():
        parser.error(f"MVNX file does not exist: {source}")
    output.parent.mkdir(parents=True, exist_ok=True)

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    counter = 0
    period = 1.0 / args.rate
    deadline = time.monotonic()
    try:
        first_metadata, first_frame = next(resampled_frames(source, args.rate))
    except StopIteration:
        print(f"error: MVNX contains no motion frames: {source}", file=sys.stderr)
        return 1
    except MvnxError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    playback = None
    enabled_at = None
    print(f"Mock Xsens replay: {source} -> {output} at {args.rate:g} Hz", flush=True)
    try:
        while running:
            enabled = control_enabled(args.control) if args.control_gated else True
            if args.control_gated and not enabled:
                metadata, frame = first_metadata, first_frame
                playback = None
                enabled_at = None
            elif args.control_gated and (enabled_at is None or time.monotonic() - enabled_at < args.start_hold):
                if enabled_at is None:
                    enabled_at = time.monotonic()
                metadata, frame = first_metadata, first_frame
            else:
                if playback is None:
                    playback = iter(resampled_frames(source, args.rate))
                try:
                    metadata, frame = next(playback)
                except StopIteration:
                    if not args.loop:
                        break
                    playback = iter(resampled_frames(source, args.rate))
                    metadata, frame = next(playback)
            now_ns = time.monotonic_ns()
            live = live_frame_from_mvnx(metadata, frame, counter, now_ns, args.rate)
            temporary = output.with_suffix(output.suffix + ".tmp")
            temporary.write_bytes(encode_live_frame_json(live))
            os.replace(temporary, output)
            counter += 1
            deadline += period
            time.sleep(max(0.0, deadline - time.monotonic()))
    except MvnxError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

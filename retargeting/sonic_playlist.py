"""Stream every exported SONIC reference through one simulation session."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Sequence

from .sonic_stream import SonicStreamError, load_trajectory, pack_protocol_v1


@dataclass(frozen=True)
class PlaylistEntry:
    name: str
    directory: Path
    frame_count: int
    duration_seconds: float
    message: bytes


def load_playlist(root: str | Path) -> list[PlaylistEntry]:
    root = Path(root).expanduser().resolve()
    metadata_paths = sorted(
        root.glob("*/sonic_reference/trajectory_metadata.json"),
        key=lambda path: path.parent.parent.name.casefold(),
    )
    if not metadata_paths:
        raise SonicStreamError(f"no SONIC references found under: {root}")

    entries = []
    for metadata_path in metadata_paths:
        directory = metadata_path.parent
        joint_pos, joint_vel, body_quat = load_trajectory(directory)
        entries.append(
            PlaylistEntry(
                name=directory.parent.name,
                directory=directory,
                frame_count=len(joint_pos),
                duration_seconds=(len(joint_pos) - 1) / 50.0,
                message=pack_protocol_v1(joint_pos, joint_vel, body_quat),
            )
        )
    return entries


def stream_playlist(
    entries: Sequence[PlaylistEntry],
    bind: str,
    startup_delay: float,
    gap_seconds: float,
) -> None:
    try:
        import zmq
    except ImportError as error:
        raise SonicStreamError("pyzmq is required; install the SONIC publisher environment") from error

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    try:
        socket.bind(bind)
        time.sleep(startup_delay)
        for index, entry in enumerate(entries, start=1):
            print(
                f"[{index}/{len(entries)}] {entry.name}: {entry.frame_count} frames, "
                f"{entry.duration_seconds:.2f} s",
                flush=True,
            )
            socket.send(entry.message)
            time.sleep(entry.duration_seconds)
            if index != len(entries):
                time.sleep(gap_seconds)
    except zmq.ZMQError as error:
        raise SonicStreamError(f"ZMQ playlist failed at {bind}: {error}") from error
    finally:
        socket.close()
        context.term()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "root",
        nargs="?",
        type=Path,
        default=Path("outputs/xsens-g1-batch"),
        help="batch output root (default: outputs/xsens-g1-batch)",
    )
    parser.add_argument("--bind", default="tcp://127.0.0.1:5556")
    parser.add_argument("--startup-delay", type=float, default=1.0)
    parser.add_argument("--gap", type=float, default=3.0, help="seconds between clips")
    parser.add_argument("--dry-run", action="store_true", help="validate and list clips without publishing")
    args = parser.parse_args(argv)
    if args.startup_delay < 0 or args.gap < 0:
        parser.error("delays must be non-negative")

    try:
        entries = load_playlist(args.root)
        motion_seconds = sum(entry.duration_seconds for entry in entries)
        total_seconds = motion_seconds + args.gap * max(0, len(entries) - 1)
        print(
            f"Validated {len(entries)} clips: {motion_seconds:.2f} s motion, "
            f"{total_seconds:.2f} s including gaps",
            flush=True,
        )
        if args.dry_run:
            for index, entry in enumerate(entries, start=1):
                print(
                    f"[{index}/{len(entries)}] {entry.name}: "
                    f"{entry.frame_count} frames, {entry.duration_seconds:.2f} s"
                )
        else:
            stream_playlist(entries, args.bind, args.startup_delay, args.gap)
            print("Playlist complete", flush=True)
    except SonicStreamError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

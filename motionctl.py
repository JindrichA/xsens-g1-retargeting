#!/usr/bin/env python3
"""Inspect and validate recorded Xsens MVNX motion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from retargeting.mvnx import MvnxError, iter_frames, read_metadata, validate


def print_inspection(path: Path) -> None:
    metadata = read_metadata(path)
    result = validate(path)
    print(f"file: {path}")
    print(f"rate: {metadata.frame_rate} Hz")
    print(f"frames: {result['frames']}")
    print(f"duration: {result['duration_seconds']:.3f} s")
    print(f"configuration: {metadata.configuration} ({metadata.processing_quality})")
    print(f"segments ({len(metadata.segment_names)}): {', '.join(metadata.segment_names)}")
    print(f"joints: {len(metadata.joint_names)}")
    print(f"sensors: {len(metadata.sensor_names)}")
    print(f"contacts: {', '.join(metadata.contact_names)}")
    print(f"finger tracking: {'yes' if metadata.finger_joint_count else 'no'}")
    print(f"source timestamp anomalies: {result['timestamp_anomalies']}")
    print(f"quaternion norm warnings: {result['quaternion_norm_warnings']}")


def sample(path: Path, frame_index: int) -> None:
    metadata = read_metadata(path)
    for frame in iter_frames(path, metadata):
        if frame.index != frame_index:
            continue
        segments = {
            name: {"position": frame.segment_positions[index], "quaternion_wxyz": frame.segment_quaternions[index]}
            for index, name in enumerate(metadata.segment_names)
        }
        print(
            json.dumps(
                {
                    "frame": frame.index,
                    "time_seconds": frame.time_seconds,
                    "segments": segments,
                    "foot_contacts": dict(zip(metadata.contact_names, frame.foot_contacts)),
                    "center_of_mass": {
                        "position": frame.center_of_mass[0:3],
                        "velocity": frame.center_of_mass[3:6],
                        "acceleration": frame.center_of_mass[6:9],
                    },
                },
                indent=2,
            )
        )
        return
    raise MvnxError(f"frame {frame_index} not found in {path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    inspect = commands.add_parser("inspect", help="show metadata and validate one MVNX recording")
    inspect.add_argument("path", type=Path)
    validate_command = commands.add_parser("validate", help="validate a file or directory of MVNX recordings")
    validate_command.add_argument("path", type=Path)
    sample_command = commands.add_parser("sample", help="print one canonical Xsens frame as JSON")
    sample_command.add_argument("path", type=Path)
    sample_command.add_argument("--frame", type=int, default=0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "inspect":
            print_inspection(args.path)
            return 0
        if args.command == "sample":
            sample(args.path, args.frame)
            return 0
        paths = sorted(args.path.rglob("*.mvnx")) if args.path.is_dir() else [args.path]
        if not paths:
            raise MvnxError(f"no MVNX files found under {args.path}")
        failed = False
        for path in paths:
            try:
                result = validate(path)
                print(
                    f"OK {path.name}: {result['frames']} frames, "
                    f"{result['duration_seconds']:.3f} s, "
                    f"{result['timestamp_anomalies']} timestamp warnings"
                )
            except MvnxError as exc:
                failed = True
                print(f"FAIL {path}: {exc}", file=sys.stderr)
        return int(failed)
    except MvnxError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

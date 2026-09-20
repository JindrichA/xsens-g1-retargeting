#!/usr/bin/env python3
"""Export all recorded MVNX motions through the Isaac G1 retargeter."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import shlex
import subprocess
import sys
import unicodedata


ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT = ROOT / "xsens_TJ_08072026" / "MVNX export"
DEFAULT_OUTPUT = ROOT / "outputs" / "xsens-g1-batch"
TASK_NAMES = ("torso", "left_wrist", "right_wrist", "left_ankle", "right_ankle")


def slugify(value: str) -> str:
    value = unicodedata.normalize("NFKD", value).encode("ascii", "ignore").decode("ascii").lower()
    return re.sub(r"[^a-z0-9]+", "-", value).strip("-") or "motion"


def select_sources(input_path: Path, recording_names: list[str]) -> list[Path]:
    if input_path.is_dir():
        sources = sorted(input_path.rglob("*.mvnx"), key=lambda path: path.name.casefold())
    elif input_path.is_file() and input_path.suffix.lower() == ".mvnx":
        sources = [input_path]
    else:
        raise ValueError(f"MVNX input not found: {input_path}")
    if not sources:
        raise ValueError(f"no MVNX recordings found under {input_path}")
    if not recording_names:
        return sources

    if len(recording_names) != len(set(recording_names)):
        raise ValueError("recording selections must be unique")
    by_name: dict[str, Path] = {}
    for source in sources:
        if source.name in by_name:
            raise ValueError(f"recording name is ambiguous under input: {source.name}")
        by_name[source.name] = source
    missing = [name for name in recording_names if name not in by_name]
    if missing:
        raise ValueError(f"selected recording not found: {missing[0]}")
    return [by_name[name] for name in recording_names]


def build_command(
    launcher: Path,
    source: Path,
    output: Path,
    max_frames: int,
    visible: bool,
    scale: float = 0.75,
    torso_scale: float | None = None,
    arm_scale: float | None = None,
    leg_scale: float | None = None,
    start_frame: int = 0,
) -> list[str]:
    command = [
        str(launcher),
        "--mvnx",
        str(source),
        "--diagnostics-json",
        str(output / "diagnostics.json"),
        "--diagnostics-csv",
        str(output / "diagnostics.csv"),
        "--trajectory-dir",
        str(output / "sonic_reference"),
        "--scale",
        str(scale),
    ]
    for option, value in (
        ("--torso-scale", torso_scale),
        ("--arm-scale", arm_scale),
        ("--leg-scale", leg_scale),
    ):
        if value is not None:
            command.extend([option, str(value)])
    if not visible:
        command.extend(["--headless", "--no-realtime", "--hide-xsens", "--hide-diagnostics"])
    if max_frames:
        command.extend(["--max-frames", str(max_frames)])
    if start_frame:
        command.extend(["--start-frame", str(start_frame)])
    return command


def parse_recording_starts(values: list[str]) -> dict[str, int]:
    starts = {}
    for value in values:
        name, separator, frame_text = value.rpartition("=")
        if not separator or not name:
            raise ValueError(f"invalid recording start (expected NAME.mvnx=FRAME): {value}")
        try:
            frame = int(frame_text)
        except ValueError as error:
            raise ValueError(f"invalid recording start frame: {value}") from error
        if frame < 0:
            raise ValueError(f"recording start frame cannot be negative: {value}")
        if name in starts:
            raise ValueError(f"duplicate recording start: {name}")
        starts[name] = frame
    return starts


def require_fresh_output(output: Path) -> None:
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"batch output directory is not empty: {output}")


def write_results(
    output: Path,
    input_path: Path,
    entries: list[dict[str, object]],
    retargeting_config: dict[str, object] | None = None,
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "format": "xsens-g1-batch-v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "input": str(input_path.resolve()),
        "recording_count": len(entries),
        "successful_count": sum(entry["status"] == "ok" for entry in entries),
        "failed_count": sum(entry["status"] != "ok" for entry in entries),
        "recordings": entries,
    }
    if retargeting_config is not None:
        manifest["retargeting_config"] = retargeting_config
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    fieldnames = [
        "profile",
        "scale",
        "torso_scale",
        "arm_scale",
        "leg_scale",
        "recording",
        "start_frame",
        "slug",
        "status",
        "frame_count",
        "duration_seconds",
    ]
    for task in TASK_NAMES:
        fieldnames.extend([f"{task}_rms_cm", f"{task}_max_cm"])
    fieldnames.extend(["rate_limited_frames", "near_limit_frames", "error"])
    with (output / "summary.csv").open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=fieldnames)
        writer.writeheader()
        for entry in entries:
            row: dict[str, object] = {
                "profile": retargeting_config.get("profile", "") if retargeting_config else "",
                "scale": retargeting_config.get("scale", "") if retargeting_config else "",
                "torso_scale": retargeting_config.get("torso_scale", "") if retargeting_config else "",
                "arm_scale": retargeting_config.get("arm_scale", "") if retargeting_config else "",
                "leg_scale": retargeting_config.get("leg_scale", "") if retargeting_config else "",
                "recording": entry["recording"],
                "start_frame": entry.get("start_frame", 0),
                "slug": entry["slug"],
                "status": entry["status"],
                "error": entry.get("error", ""),
            }
            if entry["status"] == "ok":
                diagnostics = entry["diagnostics"]
                trajectory = entry["trajectory"]
                row.update(
                    frame_count=trajectory["frame_count"],
                    duration_seconds=trajectory["duration_seconds"],
                    rate_limited_frames=diagnostics["rate_limited_frame_count"],
                    near_limit_frames=diagnostics["near_limit_frame_count"],
                )
                for task in TASK_NAMES:
                    row[f"{task}_rms_cm"] = diagnostics["tasks"][task]["rms_position_cm"]
                    row[f"{task}_max_cm"] = diagnostics["tasks"][task]["max_position_cm"]
            writer.writerow(row)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="MVNX file or directory")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-frames", type=int, default=0, help="Limit every clip; 0 exports complete clips")
    parser.add_argument("--start-frame", type=int, default=0, help="Default 50 Hz start frame for selected clips")
    parser.add_argument("--profile-name", default="baseline", help="Name stored for comparison reports")
    parser.add_argument("--scale", type=float, default=0.75, help="Fallback displacement scale")
    parser.add_argument("--torso-scale", type=float, help="Override torso displacement scale")
    parser.add_argument("--arm-scale", type=float, help="Override wrist displacement scale")
    parser.add_argument("--leg-scale", type=float, help="Override ankle displacement scale")
    parser.add_argument(
        "--recording",
        action="append",
        default=[],
        metavar="NAME.mvnx",
        help="Export only this recording basename; repeat to select multiple clips",
    )
    parser.add_argument(
        "--recording-start",
        action="append",
        default=[],
        metavar="NAME.mvnx=FRAME",
        help="Override the 50 Hz start frame for one recording; repeat as needed",
    )
    parser.add_argument("--visible", action="store_true", help="Show each clip in Isaac; close the window for the next")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without launching Isaac")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.max_frames < 0 or args.start_frame < 0:
        raise SystemExit("--max-frames and --start-frame cannot be negative")
    if not args.profile_name.strip():
        raise SystemExit("--profile-name cannot be empty")
    if any(
        value is not None and (not math.isfinite(value) or value <= 0)
        for value in (args.scale, args.torso_scale, args.arm_scale, args.leg_scale)
    ):
        raise SystemExit("all displacement scales must be finite and positive")
    retargeting_config = {
        "profile": args.profile_name,
        "scale": args.scale,
        "torso_scale": args.torso_scale if args.torso_scale is not None else args.scale,
        "arm_scale": args.arm_scale if args.arm_scale is not None else args.scale,
        "leg_scale": args.leg_scale if args.leg_scale is not None else args.scale,
    }
    launcher = ROOT / "run_isaac_g1_playback.sh"
    if not launcher.is_file():
        raise SystemExit(f"Isaac launcher not found: {launcher}")
    try:
        sources = select_sources(args.input, args.recording)
        recording_starts = parse_recording_starts(args.recording_start)
        selected_names = {source.name for source in sources}
        unknown_starts = recording_starts.keys() - selected_names
        if unknown_starts:
            raise ValueError(f"recording start has no selected recording: {sorted(unknown_starts)[0]}")
    except ValueError as error:
        raise SystemExit(str(error)) from error

    slugs = [slugify(source.stem) for source in sources]
    if len(slugs) != len(set(slugs)):
        raise SystemExit("recording names produce duplicate output slugs")
    commands = [
        build_command(
            launcher,
            source.resolve(),
            args.output.resolve() / slug,
            args.max_frames,
            args.visible,
            args.scale,
            args.torso_scale,
            args.arm_scale,
            args.leg_scale,
            recording_starts.get(source.name, args.start_frame),
        )
        for source, slug in zip(sources, slugs)
    ]
    if args.dry_run:
        for command in commands:
            print(shlex.join(command))
        return 0
    try:
        require_fresh_output(args.output.resolve())
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error

    entries = []
    for index, (source, slug, command) in enumerate(zip(sources, slugs, commands), start=1):
        print(f"[{index}/{len(sources)}] Exporting {source.name}", flush=True)
        result = subprocess.run(command, cwd=ROOT, check=False)
        entry: dict[str, object] = {
            "recording": source.name,
            "start_frame": recording_starts.get(source.name, args.start_frame),
            "slug": slug,
        }
        clip_output = args.output.resolve() / slug
        if result.returncode:
            entry.update(status="failed", error=f"Isaac playback exited with status {result.returncode}")
        else:
            try:
                diagnostics = json.loads((clip_output / "diagnostics.json").read_text(encoding="utf-8"))["summary"]
                trajectory = json.loads(
                    (clip_output / "sonic_reference" / "trajectory_metadata.json").read_text(encoding="utf-8")
                )
                if not isinstance(diagnostics, dict) or not isinstance(trajectory, dict):
                    raise ValueError("generated diagnostics and trajectory metadata must be objects")
                if not isinstance(diagnostics.get("frame_count"), int) or diagnostics["frame_count"] <= 0:
                    raise ValueError("diagnostics frame count must be positive")
                if (
                    trajectory.get("format") != "sonic-g1-reference-v1"
                    or trajectory.get("frame_count") != diagnostics["frame_count"]
                ):
                    raise ValueError("trajectory and diagnostics frame counts do not match")
                if trajectory.get("source") != str(source.resolve()):
                    raise ValueError("trajectory source does not match the batch recording")
                trajectory_config = trajectory.get("retargeting_config")
                if not isinstance(trajectory_config, dict) or any(
                    trajectory_config.get(field) != retargeting_config[field]
                    for field in ("scale", "torso_scale", "arm_scale", "leg_scale")
                ):
                    raise ValueError("trajectory retargeting configuration does not match the batch")
                for task in TASK_NAMES:
                    task_values = diagnostics["tasks"][task]
                    for field in (
                        "rms_position_cm",
                        "max_position_cm",
                        "rms_orientation_deg",
                        "max_orientation_deg",
                    ):
                        if (
                            isinstance(task_values[field], bool)
                            or not isinstance(task_values[field], (int, float))
                            or not math.isfinite(task_values[field])
                        ):
                            raise ValueError(f"invalid {task} {field}")
                entry.update(status="ok", diagnostics=diagnostics, trajectory=trajectory)
            except (OSError, AttributeError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                entry.update(status="failed", error=f"cannot read generated output: {exc}")
        entries.append(entry)
        write_results(args.output.resolve(), args.input, entries, retargeting_config)

    failures = sum(entry["status"] != "ok" for entry in entries)
    print(
        f"Batch complete: {len(entries) - failures}/{len(entries)} succeeded; "
        f"summary: {args.output.resolve() / 'summary.csv'}"
    )
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())

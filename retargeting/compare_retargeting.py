"""Combine retargeting batch manifests into one scale-comparison CSV."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


TASK_NAMES = ("torso", "left_wrist", "right_wrist", "left_ankle", "right_ankle")


class ComparisonError(ValueError):
    pass


def comparison_rows(manifest_paths: list[Path]) -> list[dict[str, object]]:
    rows = []
    profiles = set()
    expected_frames: dict[str, int] | None = None
    for path in manifest_paths:
        path = path.expanduser().resolve()
        try:
            manifest = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ComparisonError(f"cannot read batch manifest {path}: {error}") from error
        if not isinstance(manifest, dict) or manifest.get("format") != "xsens-g1-batch-v1":
            raise ComparisonError(f"unsupported batch manifest: {path}")
        config = manifest.get("retargeting_config")
        if not isinstance(config, dict):
            raise ComparisonError(f"manifest has no retargeting configuration: {path}")
        profile = config.get("profile")
        if not isinstance(profile, str) or not profile:
            raise ComparisonError(f"manifest has no comparison profile name: {path}")
        if profile in profiles:
            raise ComparisonError(f"duplicate comparison profile: {profile}")
        profiles.add(profile)
        for scale_name in ("scale", "torso_scale", "arm_scale", "leg_scale"):
            value = config.get(scale_name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ComparisonError(f"profile {profile!r} has invalid {scale_name}")
        entries = manifest.get("recordings")
        if not isinstance(entries, list) or not entries:
            raise ComparisonError(f"manifest contains no recordings: {path}")
        if any(not isinstance(entry, dict) or entry.get("status") != "ok" for entry in entries):
            raise ComparisonError(f"profile {profile!r} contains failed or malformed recordings")
        try:
            profile_frames = {
                entry["recording"]: entry["diagnostics"]["frame_count"] for entry in entries
            }
            if len(profile_frames) != len(entries) or any(
                isinstance(count, bool) or not isinstance(count, int) or count <= 0
                for count in profile_frames.values()
            ):
                raise ComparisonError(f"profile {profile!r} has invalid recording frame counts")
            if expected_frames is None:
                expected_frames = profile_frames
            elif profile_frames != expected_frames:
                raise ComparisonError(
                    f"profile {profile!r} does not contain the same recordings and frame counts"
                )
            for entry in entries:
                diagnostics = entry["diagnostics"]
                frame_count = diagnostics["frame_count"]
                rate_limited = diagnostics["rate_limited_frame_count"]
                near_limit = diagnostics["near_limit_frame_count"]
                for name, count in (
                    ("rate_limited_frame_count", rate_limited),
                    ("near_limit_frame_count", near_limit),
                ):
                    if isinstance(count, bool) or not isinstance(count, int) or not 0 <= count <= frame_count:
                        raise ComparisonError(f"profile {profile!r} has invalid {name}")
                row: dict[str, object] = {
                    "profile": profile,
                    "recording": entry["recording"],
                    "scale": config["scale"],
                    "torso_scale": config["torso_scale"],
                    "arm_scale": config["arm_scale"],
                    "leg_scale": config["leg_scale"],
                    "frame_count": frame_count,
                    "rate_limited_frames": rate_limited,
                    "near_limit_frames": near_limit,
                    "rate_limited_fraction": rate_limited / frame_count,
                    "near_limit_fraction": near_limit / frame_count,
                    "manifest": str(path),
                }
                for task in TASK_NAMES:
                    task_values = diagnostics["tasks"][task]
                    for source_field, output_field in (
                        ("rms_position_cm", f"{task}_rms_cm"),
                        ("max_position_cm", f"{task}_max_cm"),
                        ("rms_orientation_deg", f"{task}_rms_deg"),
                        ("max_orientation_deg", f"{task}_max_deg"),
                    ):
                        value = task_values[source_field]
                        if (
                            isinstance(value, bool)
                            or not isinstance(value, (int, float))
                            or not math.isfinite(value)
                            or value < 0
                        ):
                            raise ComparisonError(
                                f"profile {profile!r} has invalid {task} {source_field}"
                            )
                        row[output_field] = value
                rows.append(row)
        except (AttributeError, KeyError, TypeError, ZeroDivisionError) as error:
            raise ComparisonError(f"malformed profile {profile!r} in {path}: {error}") from error
    return rows


def write_comparison(path: Path, rows: list[dict[str, object]]) -> Path:
    if not rows:
        raise ComparisonError("comparison contains no successful recordings")
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "profile",
        "recording",
        "scale",
        "torso_scale",
        "arm_scale",
        "leg_scale",
        "frame_count",
        *(
            field
            for task in TASK_NAMES
            for field in (
                f"{task}_rms_cm",
                f"{task}_max_cm",
                f"{task}_rms_deg",
                f"{task}_max_deg",
            )
        ),
        "rate_limited_frames",
        "near_limit_frames",
        "rate_limited_fraction",
        "near_limit_fraction",
        "manifest",
    ]
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifests", nargs="+", type=Path, help="batch manifest.json files")
    parser.add_argument("--output", type=Path, default=Path("outputs/retargeting-scale-comparison.csv"))
    args = parser.parse_args(argv)
    try:
        rows = comparison_rows(args.manifests)
        output = write_comparison(args.output, rows)
    except ComparisonError as error:
        parser.error(str(error))
    print(f"Wrote {len(rows)} comparison rows: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Run and rank a bounded multi-clip retargeting scale sweep."""

from __future__ import annotations

import argparse
import csv
from itertools import product
import json
import math
from pathlib import Path
import shlex
import subprocess
import sys

from .compare_retargeting import TASK_NAMES, comparison_rows, write_comparison


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "xsens_TJ_08072026" / "MVNX export"
DEFAULT_OUTPUT = ROOT / "outputs" / "retargeting-scale-sweep"
CHAINS = {
    "torso": ("torso",),
    "arms": ("left_wrist", "right_wrist"),
    "legs": ("left_ankle", "right_ankle"),
}
POSITION_REGRESSION_FLOOR_CM = 0.1
ORIENTATION_REGRESSION_FLOOR_DEG = 0.5


class SweepError(ValueError):
    pass


def parse_scales(value: str) -> list[float]:
    try:
        scales = [float(item.strip()) for item in value.split(",")]
    except ValueError as error:
        raise SweepError(f"invalid scale list: {value}") from error
    if not scales or any(not math.isfinite(scale) or scale <= 0 for scale in scales):
        raise SweepError("scale lists must contain finite positive values")
    return list(dict.fromkeys(scales))


def _scale_label(value: float) -> str:
    return format(value, ".6g").replace("-", "m").replace(".", "p")


def build_profiles(
    baseline: float,
    torso_scales: list[float],
    arm_scales: list[float],
    leg_scales: list[float],
    cartesian: bool = False,
) -> list[dict[str, float | str]]:
    if not math.isfinite(baseline) or baseline <= 0:
        raise SweepError("baseline scale must be finite and positive")
    for values in (torso_scales, arm_scales, leg_scales):
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise SweepError("profile scales must be finite and positive")

    baseline_profile: dict[str, float | str] = {
        "profile": "baseline",
        "scale": baseline,
        "torso_scale": baseline,
        "arm_scale": baseline,
        "leg_scale": baseline,
    }
    profiles = [baseline_profile]
    if cartesian:
        combinations = product(torso_scales, arm_scales, leg_scales)
        for torso, arm, leg in combinations:
            if (torso, arm, leg) == (baseline, baseline, baseline):
                continue
            profiles.append(
                {
                    "profile": f"t{_scale_label(torso)}-a{_scale_label(arm)}-l{_scale_label(leg)}",
                    "scale": baseline,
                    "torso_scale": torso,
                    "arm_scale": arm,
                    "leg_scale": leg,
                }
            )
    else:
        for chain, values in (
            ("torso", torso_scales),
            ("arm", arm_scales),
            ("leg", leg_scales),
        ):
            for value in values:
                if value == baseline:
                    continue
                profile = dict(baseline_profile)
                profile["profile"] = f"{chain}-{_scale_label(value)}"
                profile[f"{chain}_scale"] = value
                profiles.append(profile)
    names = [str(profile["profile"]) for profile in profiles]
    if len(names) != len(set(names)):
        raise SweepError("scale values produce duplicate profile names")
    return profiles


def build_batch_command(
    profile: dict[str, float | str],
    input_path: Path,
    output_root: Path,
    recordings: list[str],
    recording_starts: list[str],
    max_frames: int,
    dry_run: bool,
) -> list[str]:
    command = [
        sys.executable,
        str(ROOT / "batch_export.py"),
        "--input",
        str(input_path),
        "--output",
        str(output_root / str(profile["profile"])),
        "--profile-name",
        str(profile["profile"]),
        "--scale",
        str(profile["scale"]),
        "--torso-scale",
        str(profile["torso_scale"]),
        "--arm-scale",
        str(profile["arm_scale"]),
        "--leg-scale",
        str(profile["leg_scale"]),
    ]
    if max_frames:
        command.extend(["--max-frames", str(max_frames)])
    for recording in recordings:
        command.extend(["--recording", recording])
    for recording_start in recording_starts:
        command.extend(["--recording-start", recording_start])
    if dry_run:
        command.append("--dry-run")
    return command


def _weighted_rms(rows: list[dict[str, object]], fields: tuple[str, ...]) -> float:
    weighted_squares = sum(
        int(row["frame_count"]) * float(row[field]) ** 2
        for row in rows
        for field in fields
    )
    sample_count = sum(int(row["frame_count"]) for row in rows) * len(fields)
    return math.sqrt(weighted_squares / sample_count)


def _ratio(value: float, baseline: float, floor: float) -> float:
    return 1.0 + (value - baseline) / max(baseline, floor)


def validate_regression_limits(max_task_regression: float, max_constraint_delta: float) -> None:
    if (
        not math.isfinite(max_task_regression)
        or not math.isfinite(max_constraint_delta)
        or max_task_regression < 0
        or max_constraint_delta < 0
    ):
        raise SweepError("regression limits must be finite and non-negative")


def rank_profiles(
    rows: list[dict[str, object]],
    baseline_profile: str = "baseline",
    max_task_regression: float = 0.05,
    max_constraint_delta: float = 0.01,
) -> list[dict[str, object]]:
    if not rows:
        raise SweepError("cannot rank an empty comparison")
    validate_regression_limits(max_task_regression, max_constraint_delta)

    grouped: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row["profile"]), []).append(row)
    if baseline_profile not in grouped:
        raise SweepError(f"baseline profile not found: {baseline_profile}")
    baseline_rows = {
        str(row["recording"]): row for row in grouped[baseline_profile]
    }

    aggregates: dict[str, dict[str, object]] = {}
    for profile, profile_rows in grouped.items():
        frame_count = sum(int(row["frame_count"]) for row in profile_rows)
        first = profile_rows[0]
        aggregate: dict[str, object] = {
            "profile": profile,
            "scale": first["scale"],
            "torso_scale": first["torso_scale"],
            "arm_scale": first["arm_scale"],
            "leg_scale": first["leg_scale"],
            "recording_count": len(profile_rows),
            "frame_count": frame_count,
            "rate_limited_fraction": sum(int(row["rate_limited_frames"]) for row in profile_rows) / frame_count,
            "near_limit_fraction": sum(int(row["near_limit_frames"]) for row in profile_rows) / frame_count,
        }
        for task in TASK_NAMES:
            aggregate[f"{task}_rms_cm"] = _weighted_rms(profile_rows, (f"{task}_rms_cm",))
            aggregate[f"{task}_rms_deg"] = _weighted_rms(profile_rows, (f"{task}_rms_deg",))
        for chain, tasks in CHAINS.items():
            aggregate[f"{chain}_rms_cm"] = _weighted_rms(
                profile_rows, tuple(f"{task}_rms_cm" for task in tasks)
            )
            aggregate[f"{chain}_rms_deg"] = _weighted_rms(
                profile_rows, tuple(f"{task}_rms_deg" for task in tasks)
            )
        aggregates[profile] = aggregate

    baseline = aggregates[baseline_profile]
    ranked = []
    for profile, aggregate in aggregates.items():
        reasons = []
        position_ratios = []
        orientation_ratios = []
        for task in TASK_NAMES:
            position_ratio = _ratio(
                float(aggregate[f"{task}_rms_cm"]),
                float(baseline[f"{task}_rms_cm"]),
                POSITION_REGRESSION_FLOOR_CM,
            )
            orientation_ratio = _ratio(
                float(aggregate[f"{task}_rms_deg"]),
                float(baseline[f"{task}_rms_deg"]),
                ORIENTATION_REGRESSION_FLOOR_DEG,
            )
            position_ratios.append(position_ratio)
            orientation_ratios.append(orientation_ratio)
        rate_delta = float(aggregate["rate_limited_fraction"]) - float(
            baseline["rate_limited_fraction"]
        )
        near_delta = float(aggregate["near_limit_fraction"]) - float(
            baseline["near_limit_fraction"]
        )
        if profile != baseline_profile:
            for row in grouped[profile]:
                recording = str(row["recording"])
                reference = baseline_rows[recording]
                for task in TASK_NAMES:
                    for suffix, label, floor in (
                        ("rms_cm", "position", POSITION_REGRESSION_FLOOR_CM),
                        ("rms_deg", "orientation", ORIENTATION_REGRESSION_FLOOR_DEG),
                    ):
                        candidate_value = float(row[f"{task}_{suffix}"])
                        reference_value = float(reference[f"{task}_{suffix}"])
                        regression = (candidate_value - reference_value) / max(
                            reference_value, floor
                        )
                        if regression > max_task_regression + 1e-12:
                            reasons.append(
                                f"{recording}/{task} {label} +{regression * 100:.1f}%"
                            )
                for count_field, label in (
                    ("rate_limited_frames", "rate-limited fraction"),
                    ("near_limit_frames", "near-limit fraction"),
                ):
                    fraction = int(row[count_field]) / int(row["frame_count"])
                    reference_fraction = int(reference[count_field]) / int(reference["frame_count"])
                    if fraction - reference_fraction > max_constraint_delta + 1e-12:
                        reasons.append(
                            f"{recording}/{label} +{fraction - reference_fraction:.3f}"
                        )

        result = dict(aggregate)
        for chain in CHAINS:
            result[f"{chain}_position_ratio"] = _ratio(
                float(aggregate[f"{chain}_rms_cm"]),
                float(baseline[f"{chain}_rms_cm"]),
                POSITION_REGRESSION_FLOOR_CM,
            )
            result[f"{chain}_orientation_ratio"] = _ratio(
                float(aggregate[f"{chain}_rms_deg"]),
                float(baseline[f"{chain}_rms_deg"]),
                ORIENTATION_REGRESSION_FLOOR_DEG,
            )
        result.update(
            position_score=sum(position_ratios) / len(position_ratios),
            orientation_score=sum(orientation_ratios) / len(orientation_ratios),
            rate_limited_delta=rate_delta,
            near_limit_delta=near_delta,
            eligible=not reasons,
            rejection_reasons="; ".join(reasons),
        )
        ranked.append(result)

    ranked.sort(
        key=lambda item: (
            not bool(item["eligible"]),
            float(item["position_score"]),
            float(item["orientation_score"]),
            str(item["profile"]),
        )
    )
    for rank, result in enumerate(ranked, start=1):
        result["rank"] = rank
    return ranked


def write_ranking(path: Path, ranking: list[dict[str, object]]) -> Path:
    if not ranking:
        raise SweepError("ranking contains no profiles")
    fields = [
        "rank",
        "eligible",
        "rejection_reasons",
        "profile",
        "scale",
        "torso_scale",
        "arm_scale",
        "leg_scale",
        "recording_count",
        "frame_count",
        "position_score",
        "orientation_score",
        "torso_position_ratio",
        "arms_position_ratio",
        "legs_position_ratio",
        "torso_orientation_ratio",
        "arms_orientation_ratio",
        "legs_orientation_ratio",
        "rate_limited_fraction",
        "rate_limited_delta",
        "near_limit_fraction",
        "near_limit_delta",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(ranking)
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--recording", action="append", default=[], metavar="NAME.mvnx")
    parser.add_argument(
        "--recording-start", action="append", default=[], metavar="NAME.mvnx=FRAME"
    )
    parser.add_argument("--max-frames", type=int, default=500)
    parser.add_argument("--baseline-scale", type=float, default=0.75)
    parser.add_argument("--torso-scales", default="0.65,0.75,0.85")
    parser.add_argument("--arm-scales", default="0.65,0.75,0.85")
    parser.add_argument("--leg-scales", default="0.65,0.75,0.85")
    parser.add_argument("--cartesian", action="store_true", help="Test every scale combination")
    parser.add_argument("--max-profiles", type=int, default=32)
    parser.add_argument("--max-task-regression", type=float, default=0.05)
    parser.add_argument("--max-constraint-delta", type=float, default=0.01)
    parser.add_argument("--rank-only", action="store_true", help="Re-rank existing profile manifests")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.max_frames < 0:
            raise SweepError("--max-frames cannot be negative")
        if args.max_profiles <= 0:
            raise SweepError("--max-profiles must be positive")
        validate_regression_limits(args.max_task_regression, args.max_constraint_delta)
        if args.rank_only and args.dry_run:
            raise SweepError("--rank-only and --dry-run cannot be combined")
        profiles = build_profiles(
            args.baseline_scale,
            parse_scales(args.torso_scales),
            parse_scales(args.arm_scales),
            parse_scales(args.leg_scales),
            args.cartesian,
        )
        if len(profiles) > args.max_profiles:
            raise SweepError(
                f"sweep has {len(profiles)} profiles, exceeding --max-profiles {args.max_profiles}"
            )
        output = args.output.expanduser().resolve()
        if not args.dry_run and not args.rank_only and output.exists() and any(output.iterdir()):
            raise SweepError(f"output directory is not empty: {output}")
    except (OSError, SweepError) as error:
        parser.error(str(error))

    commands = [
        build_batch_command(
            profile,
            args.input.expanduser().resolve(),
            output,
            args.recording,
            args.recording_start,
            args.max_frames,
            args.dry_run,
        )
        for profile in profiles
    ]
    if args.dry_run:
        for command in commands:
            print(shlex.join(command))
        print(f"Profiles: {len(profiles)}")
        return 0

    manifests = [output / str(profile["profile"]) / "manifest.json" for profile in profiles]
    if not args.rank_only:
        for index, (profile, command) in enumerate(zip(profiles, commands), start=1):
            print(f"[{index}/{len(profiles)}] Running {profile['profile']}", flush=True)
            result = subprocess.run(command, cwd=ROOT, check=False)
            if result.returncode:
                print(f"Sweep stopped: profile {profile['profile']} failed", file=sys.stderr)
                return result.returncode

    try:
        rows = comparison_rows(manifests)
        write_comparison(output / "comparison.csv", rows)
        ranking = rank_profiles(
            rows,
            max_task_regression=args.max_task_regression,
            max_constraint_delta=args.max_constraint_delta,
        )
        write_ranking(output / "ranking.csv", ranking)
        (output / "ranking.json").write_text(
            json.dumps(
                {
                    "format": "xsens-g1-scale-ranking-v1",
                    "baseline_profile": "baseline",
                    "max_task_regression": args.max_task_regression,
                    "max_constraint_delta": args.max_constraint_delta,
                    "position_regression_floor_cm": POSITION_REGRESSION_FLOOR_CM,
                    "orientation_regression_floor_deg": ORIENTATION_REGRESSION_FLOOR_DEG,
                    "profiles": ranking,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError) as error:
        print(f"Sweep outputs are invalid: {error}", file=sys.stderr)
        return 2

    best = ranking[0]
    print(
        f"Sweep complete: {len(profiles)} profiles; top eligible profile: "
        f"{best['profile']} (position score {float(best['position_score']):.4f})"
    )
    print("Kinematic ranking only; validate candidates in SONIC before changing defaults.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

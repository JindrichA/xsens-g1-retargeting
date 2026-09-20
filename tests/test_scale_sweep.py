from pathlib import Path
import unittest

from retargeting.scale_sweep import (
    SweepError,
    build_batch_command,
    build_profiles,
    parse_scales,
    rank_profiles,
    validate_regression_limits,
)


TASKS = ("torso", "left_wrist", "right_wrist", "left_ankle", "right_ankle")


def comparison_row(
    profile: str,
    position: float = 1.0,
    orientation: float = 1.0,
    rate_limited: int = 10,
    near_limit: int = 20,
) -> dict[str, object]:
    row: dict[str, object] = {
        "profile": profile,
        "recording": "clip.mvnx",
        "scale": 0.75,
        "torso_scale": 0.75,
        "arm_scale": 0.75,
        "leg_scale": 0.75,
        "frame_count": 100,
        "rate_limited_frames": rate_limited,
        "near_limit_frames": near_limit,
    }
    for task in TASKS:
        row[f"{task}_rms_cm"] = position
        row[f"{task}_rms_deg"] = orientation
    return row


class ScaleSweepTests(unittest.TestCase):
    def test_parses_unique_positive_scales(self):
        self.assertEqual(parse_scales("0.65, 0.75,0.65"), [0.65, 0.75])
        with self.assertRaisesRegex(SweepError, "finite positive"):
            parse_scales("0.75,nan")

    def test_axis_sweep_changes_one_chain_at_a_time(self):
        profiles = build_profiles(
            0.75,
            [0.65, 0.75, 0.85],
            [0.65, 0.75, 0.85],
            [0.65, 0.75, 0.85],
        )
        self.assertEqual(len(profiles), 7)
        arm = next(profile for profile in profiles if profile["profile"] == "arm-0p85")
        self.assertEqual(arm["torso_scale"], 0.75)
        self.assertEqual(arm["arm_scale"], 0.85)
        self.assertEqual(arm["leg_scale"], 0.75)

    def test_cartesian_sweep_generates_each_combination(self):
        profiles = build_profiles(0.75, [0.7, 0.75], [0.7, 0.75], [0.7, 0.75], True)
        self.assertEqual(len(profiles), 8)
        self.assertEqual(profiles[0]["profile"], "baseline")

    def test_batch_command_forwards_recording_selection(self):
        profile = build_profiles(0.75, [0.75], [0.75], [0.75])[0]
        command = build_batch_command(
            profile,
            Path("/data"),
            Path("/output"),
            ["one.mvnx", "two.mvnx"],
            ["two.mvnx=1000"],
            250,
            True,
        )
        self.assertEqual(command.count("--recording"), 2)
        self.assertEqual(command[command.index("--recording-start") + 1], "two.mvnx=1000")
        self.assertEqual(command[-1], "--dry-run")

    def test_ranks_improvement_ahead_of_baseline(self):
        baseline = comparison_row("baseline")
        improved = comparison_row("improved", position=0.9, orientation=0.99)
        ranking = rank_profiles([baseline, improved])
        self.assertEqual(ranking[0]["profile"], "improved")
        self.assertTrue(ranking[0]["eligible"])
        self.assertAlmostEqual(ranking[0]["position_score"], 0.9)

    def test_rejects_profile_when_one_task_regresses(self):
        baseline = comparison_row("baseline")
        regressed = comparison_row("regressed", position=0.9)
        regressed["left_wrist_rms_cm"] = 1.1
        ranking = rank_profiles([baseline, regressed])
        candidate = next(item for item in ranking if item["profile"] == "regressed")
        self.assertFalse(candidate["eligible"])
        self.assertIn("left_wrist position", candidate["rejection_reasons"])

    def test_rejects_constraint_regression(self):
        baseline = comparison_row("baseline")
        constrained = comparison_row("constrained", position=0.9, rate_limited=12)
        ranking = rank_profiles([baseline, constrained])
        candidate = next(item for item in ranking if item["profile"] == "constrained")
        self.assertFalse(candidate["eligible"])
        self.assertIn("rate-limited fraction", candidate["rejection_reasons"])

    def test_allows_exact_constraint_boundary_and_near_zero_noise(self):
        baseline = comparison_row("baseline", orientation=0.01)
        candidate = comparison_row("candidate", position=0.99, orientation=0.02, rate_limited=11)
        ranking = rank_profiles([baseline, candidate])
        result = next(item for item in ranking if item["profile"] == "candidate")
        self.assertTrue(result["eligible"])
        baseline_result = next(item for item in ranking if item["profile"] == "baseline")
        self.assertAlmostEqual(baseline_result["orientation_score"], 1.0)
        self.assertLess(result["orientation_score"], 1.05)

    def test_zero_baseline_score_is_stable(self):
        baseline = comparison_row("baseline", position=0.0, orientation=0.0)
        candidate = comparison_row("candidate", position=0.001, orientation=0.001)
        ranking = rank_profiles([baseline, candidate])
        baseline_result = next(item for item in ranking if item["profile"] == "baseline")
        candidate_result = next(item for item in ranking if item["profile"] == "candidate")
        self.assertEqual(baseline_result["position_score"], 1.0)
        self.assertAlmostEqual(candidate_result["position_score"], 1.01)
        self.assertAlmostEqual(candidate_result["orientation_score"], 1.002)

    def test_rejects_invalid_regression_limits_before_a_sweep(self):
        for values in ((-0.01, 0.01), (0.05, float("nan")), (float("inf"), 0.01)):
            with self.assertRaisesRegex(SweepError, "finite and non-negative"):
                validate_regression_limits(*values)


if __name__ == "__main__":
    unittest.main()

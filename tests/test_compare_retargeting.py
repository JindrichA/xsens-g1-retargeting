import csv
import json
from pathlib import Path
import tempfile
import unittest

from retargeting.compare_retargeting import ComparisonError, comparison_rows, write_comparison


def manifest(profile: str) -> dict:
    tasks = {
        name: {
            "rms_position_cm": 1.0,
            "max_position_cm": 2.0,
            "rms_orientation_deg": 3.0,
            "max_orientation_deg": 4.0,
        }
        for name in ("torso", "left_wrist", "right_wrist", "left_ankle", "right_ankle")
    }
    return {
        "format": "xsens-g1-batch-v1",
        "retargeting_config": {
            "profile": profile,
            "scale": 0.75,
            "torso_scale": 0.6,
            "arm_scale": 0.8,
            "leg_scale": 0.7,
        },
        "recordings": [
            {
                "recording": "clip.mvnx",
                "status": "ok",
                "diagnostics": {
                    "frame_count": 10,
                    "tasks": tasks,
                    "rate_limited_frame_count": 3,
                    "near_limit_frame_count": 2,
                },
            }
        ],
    }


class CompareRetargetingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def write_manifest(self, profile: str) -> Path:
        path = self.root / f"{profile}.json"
        path.write_text(json.dumps(manifest(profile)), encoding="utf-8")
        return path

    def test_combines_named_scale_profiles(self):
        rows = comparison_rows([self.write_manifest("baseline"), self.write_manifest("long-arms")])
        self.assertEqual([row["profile"] for row in rows], ["baseline", "long-arms"])
        self.assertEqual(rows[1]["arm_scale"], 0.8)
        self.assertEqual(rows[1]["left_wrist_rms_cm"], 1.0)
        self.assertEqual(rows[1]["left_wrist_rms_deg"], 3.0)
        self.assertEqual(rows[1]["rate_limited_fraction"], 0.3)

    def test_rejects_duplicate_profile_names(self):
        first = self.write_manifest("same")
        second = self.root / "second.json"
        second.write_text(json.dumps(manifest("same")), encoding="utf-8")
        with self.assertRaisesRegex(ComparisonError, "duplicate comparison profile"):
            comparison_rows([first, second])

    def test_writes_comparison_csv(self):
        rows = comparison_rows([self.write_manifest("baseline")])
        output = write_comparison(self.root / "comparison.csv", rows)
        with output.open(newline="", encoding="utf-8") as source:
            written = list(csv.DictReader(source))
        self.assertEqual(written[0]["profile"], "baseline")
        self.assertEqual(written[0]["rate_limited_frames"], "3")

    def test_rejects_mismatched_recording_sets(self):
        first = self.write_manifest("baseline")
        changed = manifest("changed")
        changed["recordings"][0]["recording"] = "other.mvnx"
        second = self.root / "changed.json"
        second.write_text(json.dumps(changed), encoding="utf-8")
        with self.assertRaisesRegex(ComparisonError, "same recording windows and frame counts"):
            comparison_rows([first, second])

    def test_rejects_mismatched_start_frames(self):
        first = self.write_manifest("baseline")
        changed = manifest("changed")
        changed["recordings"][0]["start_frame"] = 500
        second = self.root / "changed-start.json"
        second.write_text(json.dumps(changed), encoding="utf-8")
        with self.assertRaisesRegex(ComparisonError, "same recording windows"):
            comparison_rows([first, second])

    def test_rejects_malformed_successful_entry(self):
        value = manifest("broken")
        del value["recordings"][0]["diagnostics"]
        path = self.root / "broken.json"
        path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(ComparisonError, "malformed profile"):
            comparison_rows([path])

    def test_rejects_non_object_manifest(self):
        path = self.root / "list.json"
        path.write_text("[]", encoding="utf-8")
        with self.assertRaisesRegex(ComparisonError, "unsupported batch manifest"):
            comparison_rows([path])

    def test_rejects_invalid_metric(self):
        value = manifest("broken")
        value["recordings"][0]["diagnostics"]["tasks"]["torso"]["rms_position_cm"] = float("nan")
        path = self.root / "broken-metric.json"
        path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(ComparisonError, "invalid torso rms_position_cm"):
            comparison_rows([path])


if __name__ == "__main__":
    unittest.main()

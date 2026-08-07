import csv
import json
import math
from pathlib import Path
import tempfile
import unittest

from retargeting.diagnostics import DiagnosticsRecorder, FrameDiagnostics


class DiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.recorder = DiagnosticsRecorder(("wrist", "ankle"))
        self.recorder.add(
            FrameDiagnostics(
                source_frame=0,
                time_seconds=0.0,
                position_errors_m={"wrist": 0.01, "ankle": 0.02},
                orientation_errors_rad={"wrist": math.radians(10), "ankle": math.radians(20)},
                rate_limited_joints=("shoulder",),
            )
        )
        self.recorder.add(
            FrameDiagnostics(
                source_frame=5,
                time_seconds=0.02,
                position_errors_m={"wrist": 0.03, "ankle": 0.04},
                orientation_errors_rad={"wrist": math.radians(30), "ankle": math.radians(40)},
                near_limit_joints=("knee",),
            )
        )

    def test_summarizes_errors_and_constraints(self):
        summary = self.recorder.summary()
        self.assertEqual(summary["frame_count"], 2)
        self.assertAlmostEqual(summary["tasks"]["wrist"]["rms_position_cm"], math.sqrt(5))
        self.assertEqual(summary["tasks"]["ankle"]["max_position_cm"], 4.0)
        self.assertEqual(summary["rate_limited_frame_count"], 1)
        self.assertEqual(summary["near_limit_joint_counts"], {"knee": 1})

    def test_writes_json_report(self):
        path = self.recorder.write_json(Path(self.directory.name) / "report.json")
        report = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(report["summary"]["frame_count"], 2)
        self.assertEqual(report["frames"][1]["source_frame"], 5)
        self.assertAlmostEqual(report["frames"][0]["tasks"]["wrist"]["position_error_cm"], 1.0)

    def test_writes_csv_report(self):
        path = self.recorder.write_csv(Path(self.directory.name) / "report.csv")
        with path.open(newline="", encoding="utf-8") as source:
            rows = list(csv.DictReader(source))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["rate_limited_joints"], "shoulder")
        self.assertEqual(float(rows[1]["ankle_position_error_cm"]), 4.0)

    def test_rejects_mismatched_tasks(self):
        with self.assertRaisesRegex(ValueError, "do not match"):
            self.recorder.add(
                FrameDiagnostics(
                    source_frame=10,
                    time_seconds=0.04,
                    position_errors_m={"wrist": 0.0},
                    orientation_errors_rad={"wrist": 0.0},
                )
            )


if __name__ == "__main__":
    unittest.main()

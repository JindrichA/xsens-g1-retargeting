from pathlib import Path
import unittest

from retargeting.retargeting_config import (
    OUTPUT_RATE,
    TASK_LABELS,
    TASK_SEGMENTS,
    RetargetingConfig,
)


ROOT = Path(__file__).resolve().parent.parent


class RetargetingConfigTests(unittest.TestCase):
    def test_defaults_preserve_current_baseline(self):
        config = RetargetingConfig()
        self.assertEqual(config.scale, 0.75)
        self.assertEqual(config.iterations, 4)
        self.assertEqual(config.max_joint_speed, 4.0)
        self.assertEqual(config.output_rate, OUTPUT_RATE)
        self.assertEqual(config.solver, "quadprog")

    def test_rejects_invalid_limits(self):
        for values in (
            {"scale": 0},
            {"iterations": 0},
            {"max_joint_speed": 0},
            {"output_rate": 0},
            {"near_limit_margin": -0.1},
            {"solver": ""},
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                RetargetingConfig(**values)

    def test_every_task_has_one_diagnostic_label(self):
        self.assertEqual(set(TASK_LABELS), set(TASK_SEGMENTS))
        self.assertEqual(len(set(TASK_LABELS.values())), len(TASK_LABELS))

    def test_recorded_playback_imports_shared_retargeter(self):
        source = (ROOT / "retargeting/isaac_g1_playback.py").read_text(encoding="utf-8")
        self.assertIn("from retargeting.g1_retargeter import G1Retargeter", source)
        self.assertNotIn("class G1Retargeter:", source)


if __name__ == "__main__":
    unittest.main()

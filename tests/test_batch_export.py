from pathlib import Path
import unittest

import batch_export


class BatchExportTests(unittest.TestCase):
    def test_slugifies_recording_name(self):
        self.assertEqual(batch_export.slugify("Small object picking & placing.mvnx"), "small-object-picking-placing-mvnx")

    def test_builds_headless_complete_export(self):
        command = batch_export.build_command(
            Path("/repo/run.sh"),
            Path("/data/Bending forward.mvnx"),
            Path("/out/bending-forward"),
            max_frames=25,
            visible=False,
        )
        self.assertIn("--headless", command)
        self.assertIn("--trajectory-dir", command)
        self.assertEqual(command[command.index("--scale") + 1], "0.75")
        self.assertEqual(command[-2:], ["--max-frames", "25"])

    def test_visible_export_keeps_visualizations(self):
        command = batch_export.build_command(
            Path("/repo/run.sh"), Path("/data/clip.mvnx"), Path("/out/clip"), 0, True
        )
        self.assertNotIn("--headless", command)
        self.assertNotIn("--hide-xsens", command)

    def test_builds_per_body_scale_overrides(self):
        command = batch_export.build_command(
            Path("/repo/run.sh"),
            Path("/data/clip.mvnx"),
            Path("/out/clip"),
            0,
            False,
            torso_scale=0.6,
            arm_scale=0.8,
            leg_scale=0.7,
        )
        self.assertEqual(command[command.index("--torso-scale") + 1], "0.6")
        self.assertEqual(command[command.index("--arm-scale") + 1], "0.8")
        self.assertEqual(command[command.index("--leg-scale") + 1], "0.7")


if __name__ == "__main__":
    unittest.main()

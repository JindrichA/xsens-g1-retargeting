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
        self.assertEqual(command[-2:], ["--max-frames", "25"])

    def test_visible_export_keeps_visualizations(self):
        command = batch_export.build_command(
            Path("/repo/run.sh"), Path("/data/clip.mvnx"), Path("/out/clip"), 0, True
        )
        self.assertNotIn("--headless", command)
        self.assertNotIn("--hide-xsens", command)


if __name__ == "__main__":
    unittest.main()

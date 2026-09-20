from pathlib import Path
import tempfile
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

    def test_selects_requested_recordings_in_requested_order(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "a.mvnx").touch()
            (root / "b.mvnx").touch()
            sources = batch_export.select_sources(root, ["b.mvnx", "a.mvnx"])
        self.assertEqual([source.name for source in sources], ["b.mvnx", "a.mvnx"])

    def test_rejects_missing_requested_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "a.mvnx").touch()
            with self.assertRaisesRegex(ValueError, "selected recording not found"):
                batch_export.select_sources(root, ["missing.mvnx"])

    def test_parses_per_recording_start_frames(self):
        starts = batch_export.parse_recording_starts(["one.mvnx=500", "two.mvnx=1250"])
        self.assertEqual(starts, {"one.mvnx": 500, "two.mvnx": 1250})
        with self.assertRaisesRegex(ValueError, "duplicate recording start"):
            batch_export.parse_recording_starts(["one.mvnx=1", "one.mvnx=2"])

    def test_builds_nonzero_start_frame(self):
        command = batch_export.build_command(
            Path("/repo/run.sh"), Path("/data/clip.mvnx"), Path("/out/clip"), 500, False,
            start_frame=1000,
        )
        self.assertEqual(command[command.index("--start-frame") + 1], "1000")

    def test_rejects_nonempty_clip_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            clip = output / "clip"
            clip.mkdir()
            (clip / "diagnostics.json").write_text("stale", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "batch output directory is not empty"):
                batch_export.require_fresh_output(output)


if __name__ == "__main__":
    unittest.main()

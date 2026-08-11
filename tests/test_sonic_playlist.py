from pathlib import Path
import tempfile
import unittest

from retargeting.sonic_playlist import load_playlist
from retargeting.sonic_stream import SonicStreamError
from retargeting.trajectory import SONIC_G1_JOINT_ORDER, TrajectoryRecorder


class SonicPlaylistTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)

    def write_reference(self, name, frame_count):
        recorder = TrajectoryRecorder(f"{name}.mvnx")
        positions = {joint: 0.0 for joint in SONIC_G1_JOINT_ORDER}
        for frame in range(frame_count):
            recorder.add(frame * 5, positions, (0.0, 0.0, 0.75), (1.0, 0.0, 0.0, 0.0))
        recorder.write(self.root / name / "sonic_reference")

    def test_loads_references_in_name_order(self):
        self.write_reference("z-last", 3)
        self.write_reference("a-first", 2)
        entries = load_playlist(self.root)
        self.assertEqual([entry.name for entry in entries], ["a-first", "z-last"])
        self.assertEqual(entries[0].frame_count, 2)
        self.assertAlmostEqual(entries[1].duration_seconds, 0.04)
        self.assertTrue(entries[0].message.startswith(b"pose"))

    def test_rejects_empty_root(self):
        with self.assertRaisesRegex(SonicStreamError, "no SONIC references"):
            load_playlist(self.root)


if __name__ == "__main__":
    unittest.main()

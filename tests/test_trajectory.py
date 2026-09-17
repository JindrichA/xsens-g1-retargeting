import csv
import json
from pathlib import Path
import tempfile
import unittest

from retargeting.trajectory import SONIC_G1_JOINT_ORDER, TrajectoryRecorder


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.recorder = TrajectoryRecorder("motion.mvnx")

    def test_reorders_by_sonic_joint_name(self):
        positions = {name: index * 0.1 for index, name in enumerate(reversed(SONIC_G1_JOINT_ORDER))}
        self.recorder.add(0, positions, (1, 2, 3), (1, 0, 0, 0))
        expected = tuple(positions[name] for name in SONIC_G1_JOINT_ORDER)
        self.assertEqual(self.recorder.joint_positions[0], expected)

    def test_calculates_velocity_at_50_hz(self):
        initial = {name: 0.0 for name in SONIC_G1_JOINT_ORDER}
        moved = {name: 0.02 for name in SONIC_G1_JOINT_ORDER}
        self.recorder.add(0, initial, (0, 0, 0), (1, 0, 0, 0))
        self.recorder.add(5, moved, (0, 0, 0), (1, 0, 0, 0))
        self.assertEqual(self.recorder.joint_velocities[0], (0.0,) * 29)
        self.assertEqual(self.recorder.joint_velocities[1], (1.0,) * 29)

    def test_writes_sonic_reference_directory(self):
        positions = {name: 0.0 for name in SONIC_G1_JOINT_ORDER}
        self.recorder.add(0, positions, (0, 0, 0.76), (1, 0, 0, 0))
        directory = self.recorder.write(Path(self.directory.name) / "clip")
        expected = {
            "joint_pos.csv", "joint_vel.csv", "body_pos.csv", "body_quat.csv",
            "metadata.txt", "trajectory_metadata.json", "info.txt",
        }
        self.assertEqual({path.name for path in directory.iterdir()}, expected)
        with (directory / "joint_pos.csv").open(newline="", encoding="utf-8") as source:
            rows = list(csv.reader(source))
        self.assertEqual(rows[0][0], "joint_0")
        self.assertEqual(len(rows[0]), 29)
        metadata = json.loads((directory / "trajectory_metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["joint_names"], list(SONIC_G1_JOINT_ORDER))
        self.assertEqual(metadata["frame_count"], 1)

    def test_rejects_missing_joint(self):
        with self.assertRaisesRegex(ValueError, "right_wrist_yaw_joint"):
            self.recorder.add(0, {}, (0, 0, 0), (1, 0, 0, 0))

    def test_writes_retargeting_configuration(self):
        recorder = TrajectoryRecorder("motion.mvnx", retargeting_config={"arm_scale": 0.8})
        positions = {name: 0.0 for name in SONIC_G1_JOINT_ORDER}
        recorder.add(0, positions, (0, 0, 0.76), (1, 0, 0, 0))
        directory = recorder.write(Path(self.directory.name) / "configured")
        metadata = json.loads((directory / "trajectory_metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["retargeting_config"], {"arm_scale": 0.8})


if __name__ == "__main__":
    unittest.main()

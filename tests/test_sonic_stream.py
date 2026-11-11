import json
from pathlib import Path
import struct
import tempfile
import unittest

from retargeting.sonic_stream import (
    HEADER_SIZE,
    SonicStreamError,
    load_trajectory,
    pack_protocol_v1,
    select_trajectory_window,
)
from retargeting.trajectory import SONIC_G1_JOINT_ORDER, TrajectoryRecorder


class SonicStreamTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        recorder = TrajectoryRecorder("test.mvnx")
        positions = {name: index * 0.01 for index, name in enumerate(SONIC_G1_JOINT_ORDER)}
        recorder.add(0, positions, (0.0, 0.0, 0.75), (1.0, 0.0, 0.0, 0.0))
        recorder.add(5, positions, (0.0, 0.0, 0.75), (1.0, 0.0, 0.0, 0.0))
        self.directory = recorder.write(Path(self.temporary_directory.name) / "reference")

    def test_loads_exported_trajectory(self):
        joint_pos, joint_vel, body_quat = load_trajectory(self.directory)
        self.assertEqual(len(joint_pos), 2)
        self.assertEqual(len(joint_pos[0]), 29)
        self.assertEqual(body_quat[0], [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(joint_vel[0], [0.0] * 29)

    def test_packs_current_sonic_wire_format(self):
        joint_pos, joint_vel, body_quat = load_trajectory(self.directory)
        message = pack_protocol_v1(joint_pos, joint_vel, body_quat, frame_indices=(100, 101))
        self.assertEqual(message[:4], b"pose")
        header = json.loads(message[4:4 + HEADER_SIZE].rstrip(b"\0"))
        self.assertEqual(header["v"], 1)
        self.assertEqual(header["count"], 2)
        self.assertEqual(header["fields"][2]["name"], "body_quat_w")
        payload = message[4 + HEADER_SIZE:]
        expected_bytes = 2 * 29 * 4 + 2 * 29 * 4 + 2 * 4 * 4 + 2 * 8 + 1
        self.assertEqual(len(payload), expected_bytes)
        self.assertEqual(struct.unpack_from("<f", payload)[0], joint_pos[0][0])
        frame_index_offset = 2 * 29 * 4 + 2 * 29 * 4 + 2 * 4 * 4
        self.assertEqual(struct.unpack_from("<2q", payload, frame_index_offset), (100, 101))
        self.assertEqual(payload[-1], 0)

    def test_rejects_non_monotonic_frame_indexes(self):
        joint_pos, joint_vel, body_quat = load_trajectory(self.directory)
        with self.assertRaisesRegex(SonicStreamError, "strictly increasing"):
            pack_protocol_v1(joint_pos, joint_vel, body_quat, frame_indices=(10, 10))

    def test_packer_rejects_non_normalized_quaternion(self):
        joint_pos, joint_vel, body_quat = load_trajectory(self.directory)
        body_quat[1] = [2.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(SonicStreamError, "not normalized"):
            pack_protocol_v1(joint_pos, joint_vel, body_quat)

    def test_rejects_wrong_joint_order(self):
        path = self.directory / "trajectory_metadata.json"
        metadata = json.loads(path.read_text(encoding="utf-8"))
        metadata["joint_names"].reverse()
        path.write_text(json.dumps(metadata), encoding="utf-8")
        with self.assertRaisesRegex(SonicStreamError, "IsaacLab order"):
            load_trajectory(self.directory)

    def test_rejects_non_normalized_quaternion(self):
        path = self.directory / "body_quat.csv"
        path.write_text("w,x,y,z\n2,0,0,0\n1,0,0,0\n", encoding="utf-8")
        with self.assertRaisesRegex(SonicStreamError, "not normalized"):
            load_trajectory(self.directory)

    def test_selects_bounded_trajectory_window(self):
        joint_pos, joint_vel, body_quat = load_trajectory(self.directory)
        selected = select_trajectory_window(joint_pos, joint_vel, body_quat, 1, 1)
        self.assertEqual(selected[0], joint_pos[1:2])
        self.assertEqual(selected[1], joint_vel[1:2])
        self.assertEqual(selected[2], body_quat[1:2])

    def test_rejects_trajectory_window_past_end(self):
        joint_pos, joint_vel, body_quat = load_trajectory(self.directory)
        with self.assertRaisesRegex(SonicStreamError, "exceeds 2 frames"):
            select_trajectory_window(joint_pos, joint_vel, body_quat, 1, 2)


if __name__ == "__main__":
    unittest.main()

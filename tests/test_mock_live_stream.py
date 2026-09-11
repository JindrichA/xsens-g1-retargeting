import json
from pathlib import Path
import tempfile
import unittest

from retargeting.live_frame import CANONICAL_SEGMENTS, decode_live_frame_json, encode_live_frame_json
from retargeting.mock_live_stream import control_enabled, live_frame_from_mvnx
from retargeting.mvnx import MvnxFrame, MvnxMetadata


class MockLiveStreamTests(unittest.TestCase):
    def test_control_gate_defaults_disabled(self):
        with tempfile.TemporaryDirectory() as directory:
            control = Path(directory) / "control.json"
            self.assertFalse(control_enabled(control))
            control.write_text(json.dumps({"enabled": True}), encoding="utf-8")
            self.assertTrue(control_enabled(control))

    def test_converts_mvnx_frame_to_canonical_live_snapshot(self):
        metadata = MvnxMetadata(
            path=Path("motion.mvnx"),
            frame_rate=240,
            segment_names=CANONICAL_SEGMENTS,
            joint_names=(),
            sensor_names=(),
            contact_names=(),
            finger_joint_count=0,
            configuration="FullBody",
            processing_quality="HD",
            recorded_at="",
        )
        positions = tuple((float(index), 0.0, 1.0) for index, _name in enumerate(CANONICAL_SEGMENTS))
        frame = MvnxFrame(
            index=24,
            time_seconds=0.1,
            source_time_ms=100,
            segment_quaternions=((1.0, 0.0, 0.0, 0.0),) * len(CANONICAL_SEGMENTS),
            segment_positions=positions,
            joint_angles_zxy=(),
            foot_contacts=(),
            center_of_mass=(),
        )

        live = live_frame_from_mvnx(metadata, frame, counter=7, now_ns=1234, rate_hz=50)
        decoded = decode_live_frame_json(encode_live_frame_json(live))

        self.assertEqual(decoded.source_sample_counter, 7)
        self.assertEqual(decoded.source_frame_time_ms, 140)
        self.assertEqual(decoded.capture_receive_steady_ns, 1234)
        self.assertEqual(decoded.segments[CANONICAL_SEGMENTS[3]].position, positions[3])


if __name__ == "__main__":
    unittest.main()

import json
import math
import unittest
from retargeting.live_capture_ros import undo_driver_arm_rotation

from retargeting.live_frame import (
    CANONICAL_SEGMENTS,
    LiveFrame,
    LiveFrameError,
    LiveSegment,
    counter_advanced,
    decode_live_frame,
    encode_live_frame,
)
from retargeting.live_capture_ros import canonical_name


def valid_frame(counter=1):
    return LiveFrame(
        source_sample_counter=counter,
        source_frame_time_ms=100,
        source_receive_steady_ns=1_000_000,
        capture_receive_steady_ns=1_100_000,
        segments={
            name: LiveSegment((0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0))
            for name in CANONICAL_SEGMENTS
        },
    )


class LiveFrameTests(unittest.TestCase):
    def test_roundtrip(self):
        decoded = decode_live_frame(encode_live_frame(valid_frame(42)))
        self.assertEqual(decoded.source_sample_counter, 42)
        self.assertEqual(set(decoded.segments), set(CANONICAL_SEGMENTS))

    def test_rejects_missing_segment(self):
        frame = valid_frame()
        segments = dict(frame.segments)
        segments.pop("Head")
        with self.assertRaisesRegex(LiveFrameError, "missing Xsens segments"):
            encode_live_frame(LiveFrame(**{**frame.__dict__, "segments": segments}))

    def test_rejects_bad_quaternion(self):
        frame = valid_frame()
        segments = dict(frame.segments)
        segments["Pelvis"] = LiveSegment((0.0, 0.0, 1.0), (2.0, 0.0, 0.0, 0.0))
        with self.assertRaisesRegex(LiveFrameError, "not normalized"):
            encode_live_frame(LiveFrame(**{**frame.__dict__, "segments": segments}))

    def test_counter_advancement_and_wrap(self):
        self.assertTrue(counter_advanced(None, 10))
        self.assertTrue(counter_advanced(10, 11))
        self.assertFalse(counter_advanced(10, 10))
        self.assertFalse(counter_advanced(11, 10))
        self.assertTrue(counter_advanced(0xFFFFFFFF, 0))

    def test_undoes_driver_hand_frame_rotation(self):
        corrected_right = (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0)
        restored = undo_driver_arm_rotation("RightHand", corrected_right)
        self.assertAlmostEqual(restored[0], 1.0, places=6)
        self.assertAlmostEqual(restored[1], 0.0, places=6)

    def test_maps_ros_frame_names(self):
        self.assertEqual(canonical_name("skeleton_pelvis"), "Pelvis")
        self.assertEqual(canonical_name("skeleton_right_forearm"), "RightForeArm")
        self.assertIsNone(canonical_name("unknown"))


if __name__ == "__main__":
    unittest.main()

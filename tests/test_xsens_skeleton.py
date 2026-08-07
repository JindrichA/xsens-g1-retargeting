import math
import unittest

from retargeting.xsens_skeleton import SEGMENT_EDGES, display_positions, edge_indices


SEGMENTS = (
    "Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head",
    "RightShoulder", "RightUpperArm", "RightForeArm", "RightHand",
    "LeftShoulder", "LeftUpperArm", "LeftForeArm", "LeftHand",
    "RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToe",
    "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "LeftToe",
)


class XsensSkeletonTests(unittest.TestCase):
    def test_maps_complete_body_tree(self):
        edges = edge_indices(SEGMENTS)
        self.assertEqual(len(edges), 22)
        self.assertEqual(edges[0], (0, 1))
        self.assertEqual(SEGMENT_EDGES[-1], ("LeftFoot", "LeftToe"))

    def test_rejects_missing_segment(self):
        with self.assertRaisesRegex(ValueError, "RightToe"):
            edge_indices(tuple(name for name in SEGMENTS if name != "RightToe"))

    def test_places_initial_pelvis_at_display_origin(self):
        points = display_positions(
            ((1.0, 2.0, 3.0), (2.0, 2.0, 3.0)),
            (1.0, 2.0, 3.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.5, 1.0),
        )
        self.assertEqual(points, ((0.0, 1.5, 1.0), (1.0, 1.5, 1.0)))

    def test_removes_initial_heading(self):
        half_yaw = math.pi / 4.0
        points = display_positions(
            ((0.0, 1.0, 0.0),),
            (0.0, 0.0, 0.0),
            (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)),
            (0.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(points[0][0], 1.0)
        self.assertAlmostEqual(points[0][1], 0.0, places=7)


if __name__ == "__main__":
    unittest.main()

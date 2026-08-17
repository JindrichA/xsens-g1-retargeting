from pathlib import Path
import tempfile
import textwrap
import unittest

from retargeting.mvnx import MvnxError, iter_frames, read_metadata, validate


MVNX = """\
<?xml version="1.0"?>
<mvnx xmlns="http://www.xsens.com/mvn/mvnx" version="4">
  <subject frameRate="50" configuration="FullBody" processingQuality="HD" recDate="now">
    <segments><segment label="Pelvis"/></segments>
    <sensors><sensor label="Pelvis"/></sensors>
    <joints><joint label="jL5S1"/></joints>
    <footContactDefinition><contactDefinition label="LeftFoot_Heel" index="0"/></footContactDefinition>
    <frames segmentCount="1" sensorCount="1" jointCount="1" fingerJointCount="0">
      <frame type="normal" index="0" time="0">
        <orientation>1 0 0 0</orientation><position>1 2 3</position>
        <jointAngle>4 5 6</jointAngle><footContacts>1</footContacts>
        <centerOfMass>1 2 3 4 5 6 7 8 9</centerOfMass>
      </frame>
      <frame type="normal" index="1" time="20">
        <orientation>1 0 0 0</orientation><position>2 3 4</position>
        <jointAngle>5 6 7</jointAngle><footContacts>0</footContacts>
        <centerOfMass>1 2 3 4 5 6 7 8 9</centerOfMass>
      </frame>
    </frames>
  </subject>
</mvnx>
"""


class MvnxTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "motion.mvnx"
        self.path.write_text(textwrap.dedent(MVNX), encoding="utf-8")

    def test_reads_metadata(self):
        metadata = read_metadata(self.path)
        self.assertEqual(metadata.frame_rate, 50)
        self.assertEqual(metadata.segment_names, ("Pelvis",))
        self.assertEqual(metadata.contact_names, ("LeftFoot_Heel",))

    def test_reads_canonical_frame(self):
        frame = next(iter_frames(self.path))
        self.assertEqual(frame.segment_quaternions, ((1.0, 0.0, 0.0, 0.0),))
        self.assertEqual(frame.segment_positions, ((1.0, 2.0, 3.0),))
        self.assertEqual(frame.time_seconds, 0.0)

    def test_uses_index_for_timing_and_reports_source_anomaly(self):
        result = validate(self.path)
        self.assertEqual(result["duration_seconds"], 0.02)
        self.assertEqual(result["timestamp_anomalies"], 1)

    def test_rejects_non_contiguous_indices(self):
        self.path.write_text(textwrap.dedent(MVNX).replace('index="1"', 'index="2"'), encoding="utf-8")
        with self.assertRaisesRegex(MvnxError, "non-contiguous"):
            list(iter_frames(self.path))

    def test_reads_pose_only_export_without_optional_channels(self):
        pose_only = MVNX.replace('<joints><joint label="jL5S1"/></joints>', "")
        pose_only = pose_only.replace(
            '<footContactDefinition><contactDefinition label="LeftFoot_Heel" index="0"/></footContactDefinition>',
            "",
        )
        pose_only = pose_only.replace('<jointAngle>4 5 6</jointAngle><footContacts>1</footContacts>', "")
        pose_only = pose_only.replace('<jointAngle>5 6 7</jointAngle><footContacts>0</footContacts>', "")
        pose_only = pose_only.replace('<centerOfMass>1 2 3 4 5 6 7 8 9</centerOfMass>', "")
        self.path.write_text(textwrap.dedent(pose_only), encoding="utf-8")
        metadata = read_metadata(self.path)
        frames = list(iter_frames(self.path, metadata))
        self.assertEqual(metadata.joint_names, ())
        self.assertEqual(frames[0].joint_angles_zxy, ())
        self.assertEqual(frames[0].foot_contacts, ())
        self.assertEqual(frames[0].center_of_mass, ())


if __name__ == "__main__":
    unittest.main()

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from retargeting.sonic_console import (
    ConsoleError,
    StackController,
    StackState,
    discover_recordings,
    pack_recording,
    parse_stack_state,
)
from retargeting.sonic_stream import HEADER_SIZE
from retargeting.trajectory import SONIC_G1_JOINT_ORDER, TrajectoryRecorder


class SonicConsoleTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)

    def write_metadata(self, name, frames):
        directory = self.root / name / "sonic_reference"
        directory.mkdir(parents=True)
        (directory / "trajectory_metadata.json").write_text(
            json.dumps({"format": "sonic-g1-reference-v1", "rate_hz": 50, "frame_count": frames}),
            encoding="utf-8",
        )

    def test_discovers_recordings_in_name_order(self):
        self.write_metadata("z-last", 51)
        self.write_metadata("a-first", 101)
        recordings = discover_recordings(self.root)
        self.assertEqual([recording.name for recording in recordings], ["a-first", "z-last"])
        self.assertEqual(recordings[0].duration_seconds, 2.0)
        self.assertIsNone(recordings[0].source)

    def test_rejects_empty_recording_root(self):
        with self.assertRaisesRegex(ConsoleError, "no SONIC references"):
            discover_recordings(self.root)

    def test_pack_recording_prepends_static_standing_frames(self):
        directory = self.root / "standing" / "sonic_reference"
        recorder = TrajectoryRecorder("standing.mvnx")
        positions = {joint: 0.0 for joint in SONIC_G1_JOINT_ORDER}
        recorder.add(0, positions, (0.0, 0.0, 0.75), (1.0, 0.0, 0.0, 0.0))
        recorder.add(5, positions, (0.0, 0.0, 0.75), (1.0, 0.0, 0.0, 0.0))
        recorder.write(directory)
        message = pack_recording(discover_recordings(self.root)[0], preroll_frames=3)
        header = json.loads(message[4 : 4 + HEADER_SIZE].rstrip(b"\0"))
        self.assertEqual(header["count"], 5)

    def test_parses_active_controller_state(self):
        state = parse_stack_state(
            True,
            "mujoco:bash:0\nsonic:bash:0\n",
            "Init Done\ntransitioning to CONTROL state\nZMQ STREAMING MODE: ENABLED\n",
            "MuJoCo started\n",
        )
        self.assertTrue(state.mujoco_running)
        self.assertTrue(state.sonic_running)
        self.assertTrue(state.session_exists)
        self.assertTrue(state.running)
        self.assertTrue(state.streaming_enabled)

    def test_latest_streaming_transition_wins(self):
        state = parse_stack_state(
            True,
            "mujoco:bash:0\nsonic:bash:0\n",
            "Init Done\nZMQ STREAMING MODE: ENABLED\nZMQ STREAMING MODE: FORCE DISABLED\n",
        )
        self.assertFalse(state.streaming_enabled)

    def test_release_support_targets_one_graphical_mujoco_window(self):
        controller = StackController()
        active = StackState(True, True, True, True, True, True, True)
        with patch.object(controller, "state", return_value=active), patch(
            "retargeting.sonic_console.subprocess.run"
        ) as run:
            run.side_effect = [
                unittest.mock.Mock(returncode=0, stdout="12345\n", stderr=""),
                unittest.mock.Mock(returncode=0, stdout="", stderr=""),
            ]
            controller.release_support()
        self.assertEqual(run.call_args_list[0].args[0][1:], ["search", "--onlyvisible", "--class", "mujoco"])
        self.assertEqual(run.call_args_list[1].args[0][1:], ["key", "--window", "12345", "9"])

    def test_bundled_tmux_uses_its_terminfo_database(self):
        controller = StackController()
        if "sonic-build-deps" not in controller.tmux:
            self.skipTest("isolated tmux is not installed")
        self.assertEqual(controller.environment["TERM"], "xterm-256color")
        self.assertIn("sonic-build-deps/share/terminfo", controller.environment["TERMINFO"])


if __name__ == "__main__":
    unittest.main()

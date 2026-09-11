import unittest
from unittest.mock import patch

from retargeting.live_retarget import LiveWatchdog, emergency_stop_sonic
from retargeting.sonic_console import LivePipelineState, live_next_action
from retargeting.live_frame import CANONICAL_SEGMENTS, LIVE_STALE_MS, LiveFrame, LiveSegment


def valid_frame(counter):
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


class LiveWatchdogTests(unittest.TestCase):
    def test_default_stale_limit_tolerates_measured_stream_jitter(self):
        watchdog = LiveWatchdog()
        self.assertEqual(watchdog.stale_seconds * 1000, LIVE_STALE_MS)

    def test_duplicate_does_not_refresh_watchdog(self):
        watchdog = LiveWatchdog(stale_seconds=0.15)
        self.assertTrue(watchdog.accept(valid_frame(10), 1_000_000_000))
        self.assertFalse(watchdog.accept(valid_frame(10), 1_100_000_000))
        self.assertTrue(watchdog.stale(1_151_000_000))

    def test_new_counter_refreshes_watchdog(self):
        watchdog = LiveWatchdog(stale_seconds=0.15)
        watchdog.accept(valid_frame(10), 1_000_000_000)
        self.assertTrue(watchdog.accept(valid_frame(11), 1_100_000_000))
        self.assertFalse(watchdog.stale(1_200_000_000))

    @patch("retargeting.live_retarget.subprocess.run")
    @patch("retargeting.live_retarget.Path.is_file", return_value=True)
    def test_emergency_stop_targets_sonic_tmux_window(self, _is_file, run):
        run.return_value.returncode = 0
        self.assertTrue(emergency_stop_sonic())
        self.assertEqual(run.call_args.args[0][-3:], ["-t", "retarget-sonic-g1:sonic", "O"])

    def test_live_guide_reports_next_required_step(self):
        stopped = LivePipelineState(False, False, False, False)
        running = LivePipelineState(True, True, True, True)
        self.assertIn("STEP 1", live_next_action(False, stopped, {}))
        self.assertIn("STEP 2", live_next_action(True, stopped, {}))
        self.assertIn("STEP 3", live_next_action(True, running, {"state": "WAITING_FOR_CALIBRATION"}))
        self.assertIn("STEP 4", live_next_action(True, running, {"state": "CALIBRATED_DISABLED"}))
        self.assertIn("STOPPED", live_next_action(True, running, {"state": "STALE"}))

    def test_live_guide_identifies_mock_playback(self):
        mock = LivePipelineState(True, True, True, True, True)
        self.assertIn("Mock replay", live_next_action(True, mock, {"state": "NO_DATA"}))
        self.assertIn("MOCK ACTIVE", live_next_action(True, mock, {"state": "ENABLED"}))


if __name__ == "__main__":
    unittest.main()

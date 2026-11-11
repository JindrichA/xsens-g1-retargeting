from pathlib import Path
import contextlib
import io
import unittest
from unittest.mock import patch

import retargeting.g1_arm_sdk_playback as playback


class Motor:
    def __init__(self):
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0


class Command:
    def __init__(self):
        self.mode_pr = 0
        self.mode_machine = 0
        self.motor_cmd = [Motor() for _ in range(35)]
        self.crc = 0


class FakeCRC:
    def Crc(self, command):
        return 100 + round(command.motor_cmd[playback.WEIGHT_INDEX].q * 10)


class G1ArmSdkPlaybackTests(unittest.TestCase):
    def test_temperature_gate_is_55_c(self):
        self.assertEqual(playback.MAX_TEMPERATURE_C, 55.0)
        self.assertEqual(playback.MAX_START_TEMPERATURE_C, 45.0)

    def test_user_control_api_ids_match_current_sdk_master(self):
        self.assertEqual(playback.SWITCH_TO_USER_CTRL_API_ID, 7110)
        self.assertEqual(playback.SWITCH_TO_INTERNAL_CTRL_API_ID, 7111)
        self.assertGreater(playback.HANDOFF_SETTLE_SECONDS, 0.0)
        self.assertIn(501, playback.REGULAR_MODE_FSM_IDS)

    def test_tracking_guard_exceeds_planned_motion(self):
        self.assertEqual(playback.MAX_DISPLACEMENT_RAD, 0.55)
        self.assertEqual(playback.MAX_SPEED_RAD_S, 0.16)

    def test_physical_profile_allows_full_trimmed_source(self):
        self.assertEqual(playback.MAX_SOURCE_SECONDS, 26.0)

    def test_plan_is_relative_and_limited(self):
        positions = [[0.0] * 29, [0.0] * 29]
        positions[0][15] = 1.0
        positions[1][15] = 1.05
        deltas, summary = playback.prepare_arm_plan(positions)
        self.assertEqual(deltas[0], [0.0] * 14)
        self.assertAlmostEqual(deltas[1][0], 0.02)
        self.assertLessEqual(summary["motion_scale"], playback.MAX_MOTION_SCALE)
        self.assertLessEqual(summary["speed_scale"], playback.MAX_SPEED_SCALE)

    def test_rejects_more_aggressive_scales(self):
        positions = [[0.0] * 29, [0.0] * 29]
        with self.assertRaisesRegex(ValueError, "motion scale"):
            playback.prepare_arm_plan(positions, motion_scale=0.41)
        with self.assertRaisesRegex(ValueError, "speed scale"):
            playback.prepare_arm_plan(positions, speed_scale=0.2)

    def test_rejects_scaled_displacement_above_physical_profile_limit(self):
        positions = [[0.0] * 29, [0.0] * 29]
        positions[1][15] = 1.5
        with self.assertRaisesRegex(ValueError, "scaled arm displacement"):
            playback.prepare_arm_plan(positions)

    def test_rejects_scaled_speed_above_physical_profile_limit(self):
        positions = [[0.0] * 29, [0.0] * 29]
        positions[1][15] = 0.09
        with self.assertRaisesRegex(ValueError, "scaled arm speed"):
            playback.prepare_arm_plan(positions)

    def test_arm_command_populates_waist_hold_arms_and_weight(self):
        command = playback.build_arm_command(
            Command,
            FakeCRC(),
            [0.1] * 17,
            weight=0.5,
        )
        self.assertEqual(command.crc, FakeCRC().Crc(command))
        self.assertEqual(command.mode_pr, 0)
        self.assertEqual(command.mode_machine, 5)
        self.assertEqual(command.motor_cmd[playback.WEIGHT_INDEX].q, 0.5)
        self.assertTrue(
            all(command.motor_cmd[index].mode == 1 for index in playback.UPPER_BODY_INDICES)
        )
        self.assertTrue(
            all(command.motor_cmd[index].kp == 60.0 for index in playback.UPPER_BODY_INDICES)
        )
        self.assertTrue(
            all(command.motor_cmd[index].kd == 1.5 for index in playback.UPPER_BODY_INDICES)
        )
        untouched = set(range(35)) - set(playback.UPPER_BODY_INDICES) - {playback.WEIGHT_INDEX}
        self.assertTrue(all(command.motor_cmd[index].mode == 0 for index in untouched))
        self.assertTrue(all(command.motor_cmd[index].q == 0.0 for index in untouched))

    def test_target_rejects_urdf_limit_violation(self):
        initial = [0.0] * 17
        delta = [0.0] * 14
        delta[0] = 3.0
        with self.assertRaisesRegex(RuntimeError, "left_shoulder_pitch"):
            playback.target_from_delta(initial, delta)

    def test_target_holds_measured_waist_positions(self):
        initial = [0.1, -0.2, 0.3] + [0.0] * 14
        target = playback.target_from_delta(initial, [0.01] * 14)
        self.assertEqual(target[:3], initial[:3])
        self.assertEqual(target[3:], [0.01] * 14)

    def test_module_does_not_reference_whole_body_command_topic_or_switcher(self):
        source = Path(playback.__file__).read_text(encoding="utf-8")
        self.assertNotIn('"rt/lowcmd"', source)
        self.assertNotIn("MotionSwitcherClient", source)

    def test_execute_requires_explicit_profile_before_transport(self):
        error = io.StringIO()
        positions = [[0.0] * 29, [0.0] * 29]
        with patch.object(playback, "load_hardware_window", return_value=(positions, positions)):
            with contextlib.redirect_stderr(error):
                with self.assertRaises(SystemExit):
                    playback.main(
                        [
                            "unused",
                            "--start-frame", "0",
                            "--frame-count", "2",
                            "--execute",
                        ]
                    )
        self.assertIn("--execute requires --profile", error.getvalue())


if __name__ == "__main__":
    unittest.main()

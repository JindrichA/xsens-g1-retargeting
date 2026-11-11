import ast
from pathlib import Path
import unittest

import retargeting.g1_lowcmd_dry_run as dry_run


class Motor:
    mode = 0
    q = 0.0
    dq = 0.0
    tau = 0.0
    kp = 0.0
    kd = 0.0


class Command:
    def __init__(self):
        self.mode_pr = 0
        self.mode_machine = 0
        self.motor_cmd = [Motor() for _ in range(35)]
        self.crc = 0


class FakeCRC:
    def Crc(self, command):
        return 1000 + command.mode_machine + sum(motor.mode for motor in command.motor_cmd)


class G1LowCmdDryRunTests(unittest.TestCase):
    def test_builds_crc_valid_message_without_gains_or_torque(self):
        positions = [0.0] * 29
        velocities = [0.1] * 29
        command = dry_run.build_lowcmd(Command, FakeCRC(), positions, velocities, enabled=True)
        self.assertEqual(command.mode_machine, 5)
        self.assertEqual(command.mode_pr, 0)
        self.assertEqual(command.crc, FakeCRC().Crc(command))
        self.assertTrue(all(motor.mode == 1 for motor in command.motor_cmd[:29]))
        self.assertTrue(all(motor.mode == 0 for motor in command.motor_cmd[29:]))
        self.assertTrue(all(motor.kp == motor.kd == motor.tau == 0.0 for motor in command.motor_cmd))

    def test_rejects_target_outside_official_urdf_bound(self):
        positions = [0.0] * 29
        positions[5] = 0.3
        with self.assertRaisesRegex(ValueError, "left_ankle_roll"):
            dry_run.validate_target(positions, [0.0] * 29)

    def test_sdk_master_example_gain_profile_is_pinned_and_offline(self):
        command = dry_run.build_lowcmd(
            Command,
            FakeCRC(),
            [0.0] * 29,
            [0.0] * 29,
            enabled=True,
            gain_profile="sdk-master-example",
        )
        self.assertEqual(
            [motor.kp for motor in command.motor_cmd[:29]],
            list(dry_run.SDK_EXAMPLE_KP),
        )
        self.assertEqual(
            [motor.kd for motor in command.motor_cmd[:29]],
            list(dry_run.SDK_EXAMPLE_KD),
        )
        self.assertTrue(all(motor.kp == motor.kd == 0.0 for motor in command.motor_cmd[29:]))
        self.assertIn(dry_run.SDK_MASTER_COMMIT, dry_run.SDK_LOW_LEVEL_EXAMPLE_URL)

    def test_disabled_template_zeros_sdk_example_gains(self):
        command = dry_run.build_lowcmd(
            Command,
            FakeCRC(),
            [0.0] * 29,
            [0.0] * 29,
            enabled=False,
            gain_profile="sdk-master-example",
        )
        self.assertTrue(all(motor.mode == 0 for motor in command.motor_cmd))
        self.assertTrue(all(motor.kp == motor.kd == 0.0 for motor in command.motor_cmd))

    def test_local_watchdog_builds_disabled_templates_after_stale_input(self):
        positions = [[0.0] * 29, [0.01] * 29]
        velocities = [[0.0] * 29, [0.0] * 29]
        report = dry_run.simulate_dry_run(
            positions,
            velocities,
            Command,
            FakeCRC(),
            watchdog_ms=4.0,
            inject_stale_after_ms=0.0,
        )
        self.assertFalse(report["transmit_capability"])
        self.assertFalse(report["publisher_created"])
        self.assertEqual(report["command_count"], 11)
        self.assertGreater(report["disabled_template_count"], 0)

    def test_report_labels_sdk_example_gains_as_unapproved(self):
        report = dry_run.simulate_dry_run(
            [[0.0] * 29, [0.01] * 29],
            [[0.0] * 29, [0.0] * 29],
            Command,
            FakeCRC(),
            watchdog_ms=20.0,
            gain_profile="sdk-master-example",
        )
        self.assertFalse(report["transmit_capability"])
        self.assertEqual(report["gain_source_commit"], dry_run.SDK_MASTER_COMMIT)
        self.assertIn("not commissioning approval", report["gains"])

    def test_module_imports_no_transport_or_motion_switcher(self):
        tree = ast.parse(Path(dry_run.__file__).read_text(encoding="utf-8"))
        names = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            for alias in node.names
        }
        self.assertNotIn("ChannelPublisher", names)
        self.assertNotIn("ChannelFactoryInitialize", names)
        self.assertNotIn("MotionSwitcherClient", names)


if __name__ == "__main__":
    unittest.main()

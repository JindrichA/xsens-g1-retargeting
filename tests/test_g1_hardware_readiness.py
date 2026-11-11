import json
import ast
from pathlib import Path
import tempfile
import unittest

import retargeting.g1_dds_diagnostics as g1_dds_diagnostics
from retargeting.g1_dds_diagnostics import (
    G1_HARDWARE_JOINT_ORDER,
    G1_RECORDED_FIRMWARE,
    G1_RECORDED_VARIANT,
    summarize_low_state,
    summarize_low_states,
    validate_interface,
)
from retargeting.g1_hardware_readiness import REPORT_SCHEMA, evaluate_readiness
from retargeting.trajectory import SONIC_G1_JOINT_ORDER, TrajectoryRecorder


class G1HardwareReadinessTests(unittest.TestCase):
    def test_recorded_robot_still_blocks_and_cannot_transmit(self):
        report = evaluate_readiness(None)
        self.assertEqual(report.schema, REPORT_SCHEMA)
        self.assertEqual(report.decision, "BLOCKED")
        self.assertTrue(report.dry_run_only)
        self.assertFalse(report.transmit_capability)
        self.assertIn(G1_RECORDED_VARIANT, report.robot_variant)
        self.assertEqual(report.robot_firmware, G1_RECORDED_FIRMWARE)

    def test_valid_local_reference_still_does_not_enable_hardware(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            recorder = TrajectoryRecorder("source.mvnx")
            positions = {name: 0.0 for name in SONIC_G1_JOINT_ORDER}
            recorder.add(0, positions, [0.0, 0.0, 0.76], [1.0, 0.0, 0.0, 0.0])
            recorder.add(1, positions, [0.0, 0.0, 0.76], [1.0, 0.0, 0.0, 0.0])
            output = recorder.write(root / "sonic_reference")
            report = evaluate_readiness(output)
        trajectory = next(check for check in report.checks if check.id == "trajectory.reference")
        self.assertEqual(trajectory.status, "PASS")
        self.assertEqual(report.decision, "BLOCKED")

    def test_report_json_is_serializable(self):
        payload = evaluate_readiness(None).to_dict()
        self.assertEqual(json.loads(json.dumps(payload))["transmit_capability"], False)

    def test_lowstate_summary_is_read_only_and_uses_29_slots(self):
        class Motor:
            q = 0.1
            dq = 0.2
            tau_est = 0.3
            temperature = [35, 36]
            vol = 48.0
            motorstate = 0

        class Imu:
            quaternion = [1.0, 0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]

        class State:
            motor_state = [Motor() for _ in range(35)]
            imu_state = Imu()
            version = [1, 2]
            tick = 3
            mode_pr = 4
            mode_machine = 5
            crc = 6

        summary = summarize_low_state(State(), 6)
        self.assertTrue(summary["read_only"])
        self.assertFalse(summary["transmit_capability"])
        self.assertTrue(summary["crc_ok"])
        self.assertEqual(len(summary["joints"]), len(G1_HARDWARE_JOINT_ORDER))
        self.assertEqual(summary["firmware_recorded"], "V1.5.1.1")
        self.assertEqual(summary["firmware_verification"], "OPERATOR_RECORDED_NOT_DDS_VERIFIED")
        self.assertEqual(summary["robot_variant"], "G1 EDU+")
        self.assertEqual(summary["robot_hands"], "dummy hands")

    def test_multi_sample_summary_requires_advancing_tick_and_valid_crc(self):
        class Motor:
            q = 0.1
            dq = 0.2
            tau_est = 0.3
            temperature = [35, 36]
            vol = 48.0
            motorstate = 0

        class Imu:
            quaternion = [1.0, 0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]

        class State:
            motor_state = [Motor() for _ in range(35)]
            imu_state = Imu()
            version = [1, 2]
            mode_pr = 4
            mode_machine = 5

            def __init__(self, tick, crc):
                self.tick = tick
                self.crc = crc

        states = [State(10, 100), State(11, 101), State(12, 102)]
        summary = summarize_low_states(states, [100, 101, 102])
        self.assertEqual(summary["diagnostic_status"], "PASS")
        self.assertTrue(summary["observation"]["tick_advancing"])
        self.assertTrue(summary["observation"]["crc_all_ok"])

        failed = summarize_low_states([State(10, 100), State(10, 101)], [100, 999])
        self.assertEqual(failed["diagnostic_status"], "FAIL")
        self.assertFalse(failed["observation"]["tick_advancing"])
        self.assertFalse(failed["observation"]["crc_all_ok"])

        changing_mode = State(11, 101)
        changing_mode.mode_machine = 6
        failed = summarize_low_states([State(10, 100), changing_mode], [100, 101])
        self.assertEqual(failed["diagnostic_status"], "FAIL")
        self.assertFalse(failed["observation"]["mode_machine_stable"])

    def test_dds_worker_imports_no_command_publisher(self):
        tree = ast.parse(Path(g1_dds_diagnostics.__file__).read_text(encoding="utf-8"))
        imported_names = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            for alias in node.names
        }
        self.assertNotIn("ChannelPublisher", imported_names)
        self.assertNotIn("LowCmd_", imported_names)

    def test_physical_interface_rejects_loopback(self):
        with self.assertRaisesRegex(ValueError, "non-loopback"):
            validate_interface("lo")
        with self.assertRaisesRegex(ValueError, "wired Ethernet"):
            validate_interface("wlan0")


if __name__ == "__main__":
    unittest.main()

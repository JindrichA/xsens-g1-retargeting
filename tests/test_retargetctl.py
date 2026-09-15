from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import retargetctl


class RetargetctlTests(unittest.TestCase):
    def write_config(self, value):
        directory = tempfile.TemporaryDirectory()
        path = Path(directory.name) / "stacks.json"
        path.write_text(json.dumps(value), encoding="utf-8")
        self.addCleanup(directory.cleanup)
        return path

    def root_file(self, name, content):
        directory = tempfile.TemporaryDirectory()
        path = Path(directory.name) / name
        path.write_text(content, encoding="utf-8")
        self.addCleanup(directory.cleanup)
        return path

    def test_loads_valid_stack(self):
        path = self.write_config(
            {"stacks": {"demo": {"components": [{"name": "one", "cwd": ".", "command": "true"}]}}}
        )
        self.assertIn("demo", retargetctl.load_stacks(path))

    def test_rejects_duplicate_component_names(self):
        component = {"name": "same", "cwd": ".", "command": "true"}
        path = self.write_config({"stacks": {"demo": {"components": [component, component]}}})
        with self.assertRaisesRegex(retargetctl.ConfigError, "duplicate"):
            retargetctl.load_stacks(path)

    def test_rejects_invalid_runtime_boundary_metadata(self):
        component = {
            "name": "source",
            "cwd": ".",
            "command": "true",
            "inputs": "not-a-list",
        }
        path = self.write_config({"stacks": {"demo": {"components": [component]}}})
        with self.assertRaisesRegex(retargetctl.ConfigError, "invalid inputs"):
            retargetctl.load_stacks(path)

    def test_expands_projects_override(self):
        with patch.dict("os.environ", {"RETARGET_PROJECTS_ROOT": "/tmp/projects"}):
            self.assertEqual(retargetctl.expand("{projects}/repo"), "/tmp/projects/repo")

    def test_dry_run_does_not_invoke_tmux(self):
        stack = {
            "profile": "simulation",
            "components": [{"name": "one", "cwd": "/tmp", "command": "printf ok"}],
            "session": "demo",
        }
        with patch.object(retargetctl, "tmux") as mocked_tmux:
            self.assertEqual(retargetctl.start_stack("demo", stack, False, True), 0)
            mocked_tmux.assert_not_called()

    def test_start_rejects_non_simulation_profile(self):
        stack = {
            "profile": "hardware",
            "components": [{"name": "one", "cwd": "/tmp", "command": "true"}],
        }
        with self.assertRaisesRegex(retargetctl.ConfigError, "unsupported profile"):
            retargetctl.start_stack("demo", stack, False, True)

    def test_start_requires_explicit_profile(self):
        stack = {"components": [{"name": "one", "cwd": "/tmp", "command": "true"}]}
        with self.assertRaisesRegex(retargetctl.ConfigError, "profile 'missing'"):
            retargetctl.start_stack("demo", stack, False, True)

    def test_check_reports_missing_required_file_content(self):
        required_file = self.root_file("message.idl", "uint32 source_sample_counter")
        stack = {
            "requirements": {
                "files": [{"path": str(required_file), "contains": ["source_frame_time_ms"]}]
            },
            "components": [{"name": "one", "cwd": "/tmp", "command": "true"}],
        }
        output = StringIO()
        with redirect_stdout(output):
            missing = retargetctl.check_stack("demo", stack)
        self.assertEqual(len(missing), 1)
        self.assertIn("source_frame_time_ms", missing[0])

    def test_check_without_stack_checks_every_stack(self):
        stacks = {
            "first": {"components": [{"name": "one", "cwd": "/tmp", "command": "true"}]},
            "second": {"components": [{"name": "two", "cwd": "/tmp", "command": "true"}]},
        }
        path = self.write_config({"stacks": stacks})
        with patch.object(retargetctl, "check_stack", side_effect=[[], ["missing"]]) as check:
            self.assertEqual(retargetctl.main(["--config", str(path), "check"]), 1)
            self.assertEqual(check.call_count, 2)

    def test_describe_reports_runtime_boundaries_without_tmux(self):
        stack = {
            "description": "demo stack",
            "profile": "simulation",
            "maturity": "experimental",
            "dependencies": [
                {"name": "sibling", "kind": "repository", "location": "{projects}/sibling"}
            ],
            "components": [
                {
                    "name": "source",
                    "cwd": "{root}",
                    "command": "true",
                    "runtime": "ROS 2 Jazzy",
                    "role": "produce frames",
                    "inputs": ["UDP 9763"],
                    "outputs": ["/link_states"],
                }
            ],
        }
        path = self.write_config({"stacks": {"demo": stack}})
        output = StringIO()
        with patch.object(retargetctl, "tmux") as mocked_tmux, redirect_stdout(output):
            self.assertEqual(retargetctl.main(["--config", str(path), "describe", "demo"]), 0)
        mocked_tmux.assert_not_called()
        self.assertIn("profile: simulation", output.getvalue())
        self.assertIn("runtime: ROS 2 Jazzy", output.getvalue())
        self.assertIn("inputs: UDP 9763", output.getvalue())
        self.assertIn("prerequisite checks:", output.getvalue())

    def test_checked_in_stacks_document_runtime_boundaries(self):
        stacks = retargetctl.load_stacks(retargetctl.DEFAULT_CONFIG)
        for name, stack in stacks.items():
            with self.subTest(stack=name):
                self.assertTrue(stack.get("profile"))
                self.assertTrue(stack.get("maturity"))
                self.assertTrue(stack.get("dependencies"))
                for component in stack["components"]:
                    self.assertTrue(component.get("runtime"))
                    self.assertTrue(component.get("role"))
                    self.assertIsInstance(component.get("inputs"), list)
                    self.assertIsInstance(component.get("outputs"), list)

    def test_status_reports_process_exit_as_degraded(self):
        stack = {
            "session": "demo",
            "components": [{"name": "source", "cwd": "/tmp", "command": "true"}],
        }
        panes = Mock(stdout="source: bash dead=0\n")
        failed_log = Mock(stdout="[retargetctl] process exited with status 1\n")
        output = StringIO()
        with patch.object(retargetctl, "session_exists", return_value=True), patch.object(
            retargetctl, "tmux", side_effect=[panes, failed_log]
        ), redirect_stdout(output):
            self.assertEqual(retargetctl.show_status("demo", stack), 1)
        self.assertIn("demo: degraded", output.getvalue())
        self.assertIn("failed components: source (process exited)", output.getvalue())

    def test_status_reports_missing_component_pane(self):
        stack = {
            "session": "demo",
            "components": [{"name": "source", "cwd": "/tmp", "command": "true"}],
        }
        output = StringIO()
        with patch.object(retargetctl, "session_exists", return_value=True), patch.object(
            retargetctl, "tmux", return_value=Mock(stdout="other: bash dead=0\n")
        ), redirect_stdout(output):
            self.assertEqual(retargetctl.show_status("demo", stack), 1)
        self.assertIn("source (missing pane)", output.getvalue())


if __name__ == "__main__":
    unittest.main()

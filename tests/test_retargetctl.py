import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import retargetctl


class RetargetctlTests(unittest.TestCase):
    def write_config(self, value):
        directory = tempfile.TemporaryDirectory()
        path = Path(directory.name) / "stacks.json"
        path.write_text(json.dumps(value), encoding="utf-8")
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

    def test_expands_projects_override(self):
        with patch.dict("os.environ", {"RETARGET_PROJECTS_ROOT": "/tmp/projects"}):
            self.assertEqual(retargetctl.expand("{projects}/repo"), "/tmp/projects/repo")

    def test_dry_run_does_not_invoke_tmux(self):
        stack = {
            "components": [{"name": "one", "cwd": "/tmp", "command": "printf ok"}],
            "session": "demo",
        }
        with patch.object(retargetctl, "tmux") as mocked_tmux:
            self.assertEqual(retargetctl.start_stack("demo", stack, False, True), 0)
            mocked_tmux.assert_not_called()

    def test_check_without_stack_checks_every_stack(self):
        stacks = {
            "first": {"components": [{"name": "one", "cwd": "/tmp", "command": "true"}]},
            "second": {"components": [{"name": "two", "cwd": "/tmp", "command": "true"}]},
        }
        path = self.write_config({"stacks": stacks})
        with patch.object(retargetctl, "check_stack", side_effect=[[], ["missing"]]) as check:
            self.assertEqual(retargetctl.main(["--config", str(path), "check"]), 1)
            self.assertEqual(check.call_count, 2)


if __name__ == "__main__":
    unittest.main()

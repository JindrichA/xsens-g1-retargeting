from pathlib import Path
import unittest
from unittest.mock import patch

from retargeting.paths import PROJECT_ROOT, projects_root


class ProjectPathTests(unittest.TestCase):
    def test_projects_root_defaults_to_repository_parent(self):
        with patch.dict("os.environ", {}, clear=True):
            self.assertEqual(projects_root(), PROJECT_ROOT.parent.resolve())

    def test_projects_root_honors_environment_override(self):
        with patch.dict("os.environ", {"RETARGET_PROJECTS_ROOT": "/tmp/sibling-projects"}):
            self.assertEqual(projects_root(), Path("/tmp/sibling-projects"))


if __name__ == "__main__":
    unittest.main()

"""Dependency-free project path resolution shared by runtime adapters."""

from __future__ import annotations

import os
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def projects_root() -> Path:
    """Return the directory containing external sibling projects."""
    return Path(os.environ.get("RETARGET_PROJECTS_ROOT", PROJECT_ROOT.parent)).expanduser().resolve()

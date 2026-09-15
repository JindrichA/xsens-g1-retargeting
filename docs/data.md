# Data And Generated Artifacts

Motion-capture files may identify participants. They are local inputs, not source
code, and must not be committed without explicit approval and provenance review.

## Local Dataset Locations

- `xsens_TJ_08072026/`: original local recording set; ignored by Git.
- `GS_data/`: additional local recording set; ignored by Git.
- `data/raw/`: preferred location for future raw datasets; ignored by Git.

Existing directories remain in place to avoid breaking local scripts. New tools
should accept an explicit input path rather than adding another hard-coded
dataset location.

## Generated Data

- `outputs/`: exports, diagnostics, manifests, and handoff bundles.
- `.cache/`: simulator-import caches.
- `screenshots/`, `logs/`, and `*.mp4`: local experiment evidence.

These locations are ignored and are not authoritative project state. Before
sharing an artifact, record the source commit, dirty-worktree status, input
dataset identifier, generator command, and checksums. Do not assume an existing
batch `manifest.json` describes every directory beside it; current batch export
can leave older clip directories in the same output root.

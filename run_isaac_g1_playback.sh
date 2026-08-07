#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_PYTHON="${ISAACSIM_PYTHON:-$HOME/isaacsim/python.sh}"

if [[ ! -x "$ISAACSIM_PYTHON" ]]; then
  printf 'Isaac Sim Python launcher not found or not executable: %s\n' "$ISAACSIM_PYTHON" >&2
  printf 'Set ISAACSIM_PYTHON to your Isaac Sim python.sh path.\n' >&2
  exit 1
fi

exec "$ISAACSIM_PYTHON" \
  "$SCRIPT_DIR/retargeting/isaac_g1_playback.py" \
  "$@"

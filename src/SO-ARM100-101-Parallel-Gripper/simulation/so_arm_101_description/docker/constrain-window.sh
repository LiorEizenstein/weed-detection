#!/usr/bin/env bash
# Resize the first window matching a name pattern to fit a single monitor.
# Usage: constrain-window.sh <name_pattern> [width] [height]
# Runs in background, waits for the window to appear, then resizes it.
set -e

NAME="${1:?Usage: constrain-window.sh <name> [width] [height]}"
W="${2:-1280}"
H="${3:-720}"

(
  # Wait up to 30s for the window to appear
  WID=$(xdotool search --sync --name "$NAME" 2>/dev/null | head -1)
  if [ -n "$WID" ]; then
    xdotool windowsize "$WID" "$W" "$H"
    xdotool windowmove "$WID" 50 50
  fi
) &

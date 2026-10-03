#!/usr/bin/env bash
# Fix exposure and white balance on both webcams (spec section 7: auto exposure off).
# Usage: set_exposure.sh [exposure] [left device] [right device]
# Control names vary by webcam; list them with: v4l2-ctl -d /dev/video0 --list-ctrls
set -euo pipefail
EXPOSURE=${1:-150}
for dev in "${2:-/dev/video0}" "${3:-/dev/video2}"; do
  v4l2-ctl -d "$dev" --set-ctrl=auto_exposure=1 2>/dev/null || v4l2-ctl -d "$dev" --set-ctrl=exposure_auto=1
  v4l2-ctl -d "$dev" --set-ctrl=exposure_time_absolute="$EXPOSURE" 2>/dev/null || v4l2-ctl -d "$dev" --set-ctrl=exposure_absolute="$EXPOSURE"
  v4l2-ctl -d "$dev" --set-ctrl=white_balance_automatic=0 2>/dev/null || v4l2-ctl -d "$dev" --set-ctrl=white_balance_temperature_auto=0 || true
  v4l2-ctl -d "$dev" --set-ctrl=focus_automatic_continuous=0 2>/dev/null || true
  echo "$dev: exposure $EXPOSURE"
done

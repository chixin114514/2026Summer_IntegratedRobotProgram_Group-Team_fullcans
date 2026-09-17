#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

python3 calibrate_phone_camera.py calibrate \
  --images calibration_images \
  --output calibration_output

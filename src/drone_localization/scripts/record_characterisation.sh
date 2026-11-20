#!/bin/bash
# Record the localisation sensors (and truth, for offline plots only).
# Usage: record_characterisation.sh [output_name]   (default: char_<date>)
set -euo pipefail

NAME="${1:-char_$(date +%Y%m%d_%H%M%S)}"

exec ros2 bag record -s mcap -o "$NAME" \
  /clock \
  /localization/gps \
  /localization/vo \
  /localization/imu \
  /evaluation/ground_truth \
  /localization/estimate

#!/usr/bin/env bash
#
# Launch the watermelon weed-detection demo and save ALL console output
# (every node: detection, arm controller, laser, field manager, plus Gazebo
# and the controllers) to a timestamped log file for later debugging.
#
# Usage:
#   ./run_demo.sh                 # normal run, logs to run_logs/run_<timestamp>.log
#   ./run_demo.sh demo.launch.py  # (default) or pass another launch file name
#
# The newest run is always also reachable at run_logs/latest.log
#
# NB: no 'set -u' — ROS 2 setup.bash references unset vars (AMENT_TRACE_SETUP_FILES)
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$WS/run_logs"
mkdir -p "$LOG_DIR"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/run_${STAMP}.log"
LAUNCH_FILE="${1:-demo.launch.py}"

# Environment for clean, greppable logs:
#   - drop ANSI colour codes so the file is plain text
#   - timestamp + severity + node name on every line
#   - unbuffered Python so logs are written as they happen (not on crash)
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{time}] [{severity}] [{name}]: {message}'
export PYTHONUNBUFFERED=1

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# Point "latest.log" at this run.
ln -sfn "$LOG_FILE" "$LOG_DIR/latest.log"

echo "=================================================================="
echo " watermelon_demo run @ ${STAMP}"
echo " launch file : ${LAUNCH_FILE}"
echo " log file    : ${LOG_FILE}"
echo " (also at)   : ${LOG_DIR}/latest.log"
echo "=================================================================="

# Run the launch, merge stderr into stdout, and tee to the log file.
# 'stdbuf -oL' keeps the tee line-buffered so the file stays current.
ros2 launch watermelon_demo "${LAUNCH_FILE}" 2>&1 | stdbuf -oL tee "${LOG_FILE}"

echo "=================================================================="
echo " run finished — full log saved to ${LOG_FILE}"
echo "=================================================================="

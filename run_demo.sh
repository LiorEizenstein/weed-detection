#!/usr/bin/env bash
#
# Launch the watermelon weed-detection demo and save ALL console output
# (every node: detection, arm controller, laser, field manager, plus Gazebo
# and the controllers) to a timestamped log file for later debugging.
#
# Usage:
#   ./run_demo.sh                                      # simulation (default)
#   ./run_demo.sh --camera                             # real camera, no robot
#   ./run_demo.sh --camera use_real_model:=false       # camera + HSV stub (no .pt)
#   ./run_demo.sh --real                               # real hardware, dry_run=true
#   ./run_demo.sh --real robot_ip:=192.168.1.100      # real hardware, custom IP
#   ./run_demo.sh --real dry_run:=false                # real hardware, fire laser
#   ./run_demo.sh demo.launch.py                       # explicit launch file
#   ./run_demo.sh demo_real.launch.py robot_ip:=X.X.X.X dry_run:=true
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

# Parse mode shortcuts; remaining args are forwarded to ros2 launch
if [[ "${1:-}" == "--camera" ]]; then
    LAUNCH_FILE="demo_camera_only.launch.py"
    SHOW_RAW_CAMERA=1
    shift
    LAUNCH_ARGS=("$@")
elif [[ "${1:-}" == "--real" ]]; then
    LAUNCH_FILE="demo_real.launch.py"
    shift
    LAUNCH_ARGS=("$@")
elif [[ "${1:-}" == *.launch.py ]]; then
    LAUNCH_FILE="$1"
    shift
    LAUNCH_ARGS=("$@")
else
    LAUNCH_FILE="demo.launch.py"
    LAUNCH_ARGS=("$@")
fi

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
echo " launch file : ${LAUNCH_FILE}${LAUNCH_ARGS[*]:+ }${LAUNCH_ARGS[*]}"
echo " log file    : ${LOG_FILE}"
echo " (also at)   : ${LOG_DIR}/latest.log"
echo "=================================================================="

# If --camera mode, open a raw image viewer in background once ROS is ready.
if [[ "${SHOW_RAW_CAMERA:-0}" == "1" ]]; then
    (sleep 5 && ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw) &
    RAW_VIEW_PID=$!
    trap 'kill "$RAW_VIEW_PID" 2>/dev/null' EXIT
fi

# Run the launch, merge stderr into stdout, and tee to the log file.
# 'stdbuf -oL' keeps the tee line-buffered so the file stays current.
ros2 launch watermelon_demo "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" 2>&1 | stdbuf -oL tee "${LOG_FILE}"

echo "=================================================================="
echo " run finished — full log saved to ${LOG_FILE}"
echo "=================================================================="

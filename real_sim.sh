#!/usr/bin/env bash
#
# Launch the real_simulation_ur5 package (Gazebo + RViz) and save ALL console
# output to a timestamped log file for later debugging.
#
# Usage:
#   ./real_sim.sh                                        # simulation (default)
#   ./real_sim.sh --real                                 # real hardware, dry_run=true
#   ./real_sim.sh --real robot_ip:=192.168.1.113         # real hardware, custom IP
#   ./real_sim.sh --real dry_run:=false                  # real hardware, fire laser
#   ./real_sim.sh model_path:=/mnt/c/Users/liore/Weed_control/version3/best.pt
#
# The newest run is always also reachable at run_logs/latest_sim.log
#
# NB: no 'set -u' — ROS 2 setup.bash references unset vars (AMENT_TRACE_SETUP_FILES)
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$WS/run_logs"
mkdir -p "$LOG_DIR"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/sim_${STAMP}.log"

# Parse mode shortcuts; remaining args are forwarded to ros2 launch
if [[ "${1:-}" == "--real" ]]; then
    LAUNCH_FILE="real.launch.py"
    shift
    LAUNCH_ARGS=("$@")
elif [[ "${1:-}" == *.launch.py ]]; then
    LAUNCH_FILE="$1"
    shift
    LAUNCH_ARGS=("$@")
else
    LAUNCH_FILE="sim.launch.py"
    LAUNCH_ARGS=("$@")
fi

# Environment for clean, greppable logs
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{time}] [{severity}] [{name}]: {message}'
export PYTHONUNBUFFERED=1

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# Point "latest_sim.log" at this run.
ln -sfn "$LOG_FILE" "$LOG_DIR/latest_sim.log"

echo "=================================================================="
echo " real_simulation_ur5 run @ ${STAMP}"
echo " launch file : ${LAUNCH_FILE}${LAUNCH_ARGS[*]:+ }${LAUNCH_ARGS[*]}"
echo " log file    : ${LOG_FILE}"
echo " (also at)   : ${LOG_DIR}/latest_sim.log"
echo "=================================================================="

# Run the launch, merge stderr into stdout, and tee to the log file.
# 'stdbuf -oL' keeps the tee line-buffered so the file stays current.
ros2 launch real_simulation_ur5 "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" 2>&1 | stdbuf -oL tee "${LOG_FILE}"

echo "=================================================================="
echo " run finished — full log saved to ${LOG_FILE}"
echo "=================================================================="

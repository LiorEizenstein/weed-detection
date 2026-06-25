#!/bin/bash
set -e

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

# Gazebo Fortress (gz-sim 6) needs plugin path to load ros2_control
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:}/opt/ros/humble/lib"

# Gazebo needs to find package meshes via GZ_SIM_RESOURCE_PATH
# ament_python installs to install/<pkg>/share, not install/share
for dir in /ws/install/*/share; do
  if [ -d "$dir" ]; then
    export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$dir"
  fi
done

# MuJoCo rendering — ensure GL libraries are findable
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}/usr/lib/x86_64-linux-gnu"

# Webots — set home if installed
if [ -d "${WEBOTS_HOME:-/usr/local/webots}" ]; then
  export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"
  export PATH="${WEBOTS_HOME}:${PATH}"
fi

# Auto-resize simulator windows to fit a single monitor (1280x720)
# This prevents GLFW/Qt apps from spanning multiple monitors via VcXsrv.
if [ -n "$DISPLAY" ] && command -v xdotool >/dev/null 2>&1; then
  for pattern in "MuJoCo" "Webots" "RViz"; do
    bash /constrain-window.sh "$pattern" 1280 720 2>/dev/null || true
  done
fi

exec "$@"

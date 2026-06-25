# SO-ARM-101 Description — ROS2 Humble build & test image
FROM ros:humble AS base

ENV DEBIAN_FRONTEND=noninteractive

# Core ROS2 tools needed for building and validating the package
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-robot-state-publisher \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        python3-colcon-common-extensions \
        python3-pytest \
        python3-pip \
        xdotool \
    && pip3 install --no-cache-dir trimesh numpy mujoco \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Copy the package into the workspace
COPY . src/so_arm_101_description/

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select so_arm_101_description --symlink-install

# Default entrypoint: source workspace and run bash
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/constrain-window.sh /constrain-window.sh
RUN chmod +x /entrypoint.sh /constrain-window.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ---------- test stage ----------
FROM base AS test

RUN . /opt/ros/humble/setup.sh && \
    . /ws/install/setup.sh && \
    colcon test --packages-select so_arm_101_description --return-code-on-test-failure

# ---------- rviz stage (needs X11 / display) ----------
FROM base AS rviz

RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "so_arm_101_description", "display.launch.py"]

# ---------- gazebo stage ----------
FROM base AS gazebo

RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        ros-humble-gz-ros2-control \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "so_arm_101_description", "sim.launch.py", "sim:=gazebo"]

# ---------- mujoco stage ----------
# Builds mujoco_ros2_control from source (dfki-ric fork, supports Humble).
# The MujocoSystemInterface plugin runs MuJoCo inside ros2_control_node.
FROM base AS mujoco

ARG MUJOCO_ROS2_REPO=https://github.com/ros-controls/mujoco_ros2_control.git
ARG MUJOCO_ROS2_BRANCH=main

RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        git \
        libgl1-mesa-dev \
        libglew-dev \
        libglfw3-dev \
        libfmt-dev \
        libtinyxml2-dev \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone mujoco_ros2_control (official ros-controls) and its dependencies:
# - ros2_control_cmake: cmake utilities not yet in Humble apt repos
# - mujoco_vendor: vendors MuJoCo C++ library for cmake find_package
RUN cd /ws/src && \
    git clone --depth 1 https://github.com/ros-controls/ros2_control_cmake.git && \
    git clone --depth 1 https://github.com/pal-robotics/mujoco_vendor.git && \
    git clone --depth 1 -b ${MUJOCO_ROS2_BRANCH} ${MUJOCO_ROS2_REPO}

# Patch: use a reasonable window size instead of full-screen on primary monitor
# (the default grabs glfwGetPrimaryMonitor() which spans all monitors via X11)
RUN sed -i \
    's|const GLFWvidmode\* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());|// patched: fixed window size|; \
     s|glfwSetWindowSize(glfwGetCurrentContext(), mode->width, mode->height);|glfwSetWindowSize(glfwGetCurrentContext(), 1280, 720);|' \
    /ws/src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system_interface.cpp

RUN . /opt/ros/humble/setup.sh && \
    rosdep update --rosdistro=humble && \
    cd /ws && \
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true && \
    colcon build --symlink-install \
        --packages-up-to mujoco_ros2_control so_arm_101_description \
        --packages-skip mujoco_ros2_control_demos mujoco_ros2_control_tests

CMD ["ros2", "launch", "so_arm_101_description", "sim.launch.py", "sim:=mujoco"]

# ---------- webots stage ----------
# Installs Webots simulator + ROS2 driver, converts URDF to Webots PROTO.
FROM base AS webots

# Install Webots (from Cyberbotics apt repo) and ROS2 driver
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        wget \
        gnupg2 \
        software-properties-common \
    && wget -qO- https://cyberbotics.com/Cyberbotics.asc | apt-key add - 2>/dev/null \
    && echo "deb https://cyberbotics.com/debian binary-amd64/" > /etc/apt/sources.list.d/cyberbotics.list \
    && apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        webots \
        ros-humble-webots-ros2 \
        libxcb-cursor0 \
    && pip3 install --no-cache-dir urdf2webots \
    && rm -rf /var/lib/apt/lists/*

ENV WEBOTS_HOME=/usr/local/webots

# Workaround: Docker on WSL2 inherits the WSL kernel name, causing
# webots_ros2_driver to detect WSL and look for webots.exe (Windows).
# Patch is_wsl() to always return False inside the container.
RUN sed -i "s/return 'microsoft-standard' in uname().release/return False/" \
    /opt/ros/humble/local/lib/python3.10/dist-packages/webots_ros2_driver/utils.py

# Generate Webots world and pre-expanded URDF for webots_ros2_driver
RUN . /opt/ros/humble/setup.sh && \
    . /ws/install/setup.sh && \
    python3 /ws/src/so_arm_101_description/scripts/setup_webots.py \
        --output-dir /ws/src/so_arm_101_description && \
    xacro /ws/src/so_arm_101_description/urdf/so_101.urdf.xacro sim_backend:=webots \
        > /ws/src/so_arm_101_description/urdf/so_101_webots.urdf && \
    colcon build --packages-select so_arm_101_description --symlink-install

CMD ["ros2", "launch", "so_arm_101_description", "sim.launch.py", "sim:=webots"]

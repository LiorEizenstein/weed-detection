# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 (Jazzy) + Gazebo Sim 8 simulation of a UR5 arm autonomously scanning a watermelon field, detecting weeds from a wrist-mounted camera, and firing a simulated Tm:YLF laser at each weed. The core algorithm is a closed-loop visual servoing controller using proportional pixel-error correction on `wrist_1` (X axis) and `wrist_2` (Y axis).

## Build & Run

```bash
# Build (from workspace root ~/ros2_ws):
colcon build --packages-select watermelon_demo
source install/setup.bash

# Simulation:
ros2 launch watermelon_demo demo.launch.py

# Real hardware (UR5 + RealSense D435):
ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=true

# Tests (all 108):
cd ~/ros2_ws
pytest src/watermelon_demo/test/ -v

# Single test file:
pytest src/watermelon_demo/test/test_centering.py -v

# Log summary:
python3 scripts/summarize_run.py run_logs/run_<timestamp>.log
python3 scripts/summarize_run.py run_logs/run_<timestamp>.log --full
```

After `colcon build`, always re-`source install/setup.bash` or nodes won't pick up changes.

## Architecture

### Node graph

```
Gazebo Sim 8 → /camera/image_raw → detection_node → /detections → arm_controller_node
                                                                          │
                                                                   /laser_fire (Bool)
                                                            ┌────────────┴────────────┐
                                                   laser_effect_node       field_manager_node
```

### Nodes (`src/watermelon_demo/watermelon_demo/`)

| Node | File | Responsibility |
|---|---|---|
| `detection_node` | `detection_node.py` | Camera → YOLO (classes: 0=watermelon, 1=weed_side, 2=weed_top) or HSV stub → `/detections` |
| `arm_controller_node` | `arm_controller_node.py` | 10 Hz state machine: INIT → SCAN_MOVE → SCANNING → WAITING_DETECTION → MOVE_TO_WEED → FIRE_LASER → FIRING. Sends joint trajectories via action client. |
| `laser_effect_node` | `laser_effect_node.py` | On `/laser_fire True`: TF ray-cast weed pixel → ground plane → RViz ARROW marker |
| `field_manager_node` | `field_manager_node.py` | Mirrors SDF plant positions as RViz markers; marks treated weeds grey |

### Centering algorithm

Proportional correction loop in `arm_controller_node`:
- Normalised pixel error: `err_x = (cx − W/2)/(W/2)`, `err_y = (cy − H/2)/(H/2)`
- `wrist_1 -= err_x × gain_x` (X correction — near-orthogonal to Y at scan nominal `wrist_2 = −π/2`)
- `wrist_2 -= err_y × gain_y` (Y correction)
- Fire when `‖err‖₂ < 0.20`
- Treated weed positions are ray-cast to world coords; detections within 0.20 m are skipped (dead-zone)

### TF tree

```
world → base_link → ... → wrist_3_link → tool0
                                       ├── camera_link  (image source, ray projection)
                                       └── laser_link   (beam visualisation origin)
```

`camera_link` and `laser_link` are defined in `urdf/ur5_with_sensors.urdf.xacro`.

## Key Configuration

**`config/demo_params.yaml`** — primary knobs:
- `use_real_model: true/false` — YOLO vs. HSV stub
- `model_path` — path to `best.pt` YOLO weights (not in git; place at `/home/lior/best.pt`)
- `scan_dwell_time` — seconds to wait at each of 21 scan poses for a detection
- `laser_fire_duration` — seconds `/laser_fire True` is held
- `save_debug_frames` — saves raw/annotated/mask frames to `run_logs/frames/`

**`config/real_params.yaml`** — real-hardware overrides (`use_sim_time: false`, camera params).

**`config/camera_params.yaml`** — `tool0`-to-camera TF offset (from easy_handeye2 calibration or manual measurement). Required before running on real hardware.

## Real Hardware Notes

- Must measure/calibrate the physical RealSense D435 offset from UR5 `tool0` flange before running. See `ROBOT_DEPLOYMENT.md` for the full checklist (easy_handeye2 ArUco calibration preferred).
- `dry_run:=true` moves the arm and centres over weeds but never publishes `/laser_fire True`.
- Joint trajectory action: `/scaled_joint_trajectory_controller/follow_joint_trajectory`

## Test Suite

108 pytest tests in `src/watermelon_demo/test/`. Tests are pure Python (no ROS spin needed for most). Key files:
- `test_centering.py` — simulates the centering loop against a calibrated kinematic model
- `test_nodes.py` — node instantiation and topic wiring
- `test_dry_run_integration.py` — integration test for dry_run mode
- `test_camera_tf.py`, `test_camera_intrinsics.py` — TF and intrinsics validation

## Docs

- `docs/architecture.md` — full node graph, topic table, TF tree
- `docs/weed_centering_control_loop.md` — state machine diagram
- `docs/calibration_log.md` — wrist gain derivation from run logs
- `ROBOT_DEPLOYMENT.md` — step-by-step real hardware deployment checklist

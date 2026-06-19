# Weed Detection & Laser Treatment — UR5 Robotic Arm Demo

**Final project, Hebrew University of Jerusalem (HUJI) — Robotics**

A ROS 2 (Jazzy) + Gazebo Sim 8 simulation of a UR5 arm autonomously scanning a
watermelon field, detecting weeds from a wrist-mounted camera, and firing a
Tm:YLF laser beam at each weed. The system implements a closed-loop visual
servoing controller that iteratively corrects the arm's joint angles until the
weed is centred in the camera frame, then fires. In a representative run the
system treated 5 out of 5 weeds with a median convergence of 4 centering steps
per weed.

---

## Problem Statement

Manual weeding in precision agriculture is labour-intensive and difficult to
scale. Robotic alternatives must solve two coupled sub-problems: detecting weeds
reliably in a cluttered scene, and positioning an effector precisely over each
target without a priori knowledge of weed locations. This project models both in
simulation: a UR5 arm sweeps a watermelon field with a wrist-mounted camera,
identifies weeds via a YOLO detector (or an HSV colour stub when the model is
unavailable), and drives the arm iteratively until the weed is within a
threshold of the image centre before firing a simulated laser.

---

## System Design

### Node graph

```
Gazebo Sim 8
  └─ /camera/image_raw ──► detection_node ──► /detections ──► arm_controller_node
                        └─► /detection_image                          │
                                                               /laser_fire (Bool)
                                                              ┌────────┴────────┐
                                                    laser_effect_node   field_manager_node
                                                    (RViz beam arrow)   (plant markers)
```

### Nodes

| Node | Responsibility |
|---|---|
| `detection_node` | Converts `/camera/image_raw` to `Detection2DArray` via YOLO or HSV stub |
| `arm_controller_node` | State-machine scan → centre → fire loop; publishes `/laser_fire` |
| `laser_effect_node` | TF ray-casts weed pixel to ground plane; draws RViz arrow marker |
| `field_manager_node` | Mirrors SDF plant positions as RViz markers; turns treated weeds grey |

Full topic table, TF tree, and parameter reference: [`docs/architecture.md`](docs/architecture.md)

---

## Centering Algorithm

The arm sweeps 21 shoulder-pan poses across ±1.5 rad (8.6° steps). On detecting
a weed, the controller enters a proportional correction loop running at 10 Hz:

1. Compute normalised pixel error: `err_x = (cx − W/2)/(W/2)`, `err_y = (cy − H/2)/(H/2)`
2. Apply simultaneous corrections: `wrist_1 −= err_x × 0.08`, `wrist_2 −= err_y × 0.10`
3. Repeat until `‖err‖₂ < 0.20` (circular fire zone), then publish `/laser_fire True`

**Why wrist_1 + wrist_2?** Early versions used `shoulder_pan` for X correction,
but `shoulder_pan` has a large Y cross-coupling (~4.7 px_norm/rad), causing
runaway in simultaneous corrections. Log analysis showed that at the scan
nominal `wrist_2 = −π/2`, `wrist_1` and `wrist_2` form a near-orthogonal pair:
`wrist_1` moves image X with negligible Y effect, and `wrist_2` moves image Y
with negligible X effect. Simultaneous dual-axis correction is therefore stable
and converges roughly twice as fast as single-axis alternation.

**Dead-zone suppression:** After each laser firing, the arm ray-casts the
treated weed pixel to world coordinates and stores the `(x, y)` ground position.
Subsequent detections within 0.20 m of a treated position are skipped,
preventing re-engagement of the same weed from adjacent scan poses.

Full state-machine diagram: [`docs/weed_centering_control_loop.md`](docs/weed_centering_control_loop.md)  
Gain derivation: [`docs/calibration_log.md`](docs/calibration_log.md)

---

## Results

Run `run_20260613_085200` (HSV stub, 21 scan poses, simulation time):

| Weed | Initial ‖err‖₂ | Centering steps | Outcome |
|------|---------------|-----------------|---------|
| 1 | 0.84 | 4 | treated |
| 2 | 0.73 | 4 | treated |
| 3 | 0.78 | 3 | treated |
| 4 | 0.82 | 4 | treated |
| 5 | 0.70 | 26 † | treated |

**Total run time:** ~86 s simulation time for 5 weeds across 14 active scan poses
(7 skipped as already-treated neighbours).

† Weed 5 had large simultaneous X and Y error (−0.52, +0.68). The single-axis
alternating scheme in use at the time of this run required 26 steps because
correcting X shifted Y and vice versa. This observation directly motivated the
dual-axis redesign described above, which resolves the coupling.

---

## Limitations & Future Work

- **No terminal state.** The arm scans indefinitely after all weeds are treated;
  a mission-complete state and operator notification are not yet implemented.
- **Single-point gain calibration.** `C_W2_Y = 1.79` is derived from one log
  observation at `wrist_2 ≈ −π/2`. A controlled multi-point calibration sweep
  would improve gain accuracy as the joint deviates from nominal.
- **Kinematic linearisation.** The dual-axis model is a first-order
  approximation valid near the scan nominal. A full 2×2 Jacobian correction
  would remain accurate across the centering range.
- **HSV detection.** The colour stub is tuned for Gazebo's lighting and the
  specific brown weed meshes. Real-world deployment requires the trained YOLO
  model and re-tuning for field conditions.
- **No force/collision awareness.** Joint trajectories are sent open-loop with
  no torque monitoring; the arm will not detect unexpected contact.

---

## Build & Run

**Dependencies:** ROS 2 Jazzy, Gazebo Sim 8, `ur_simulation_gz`, `ros_gz_bridge`,
`ros2_control`, `vision_msgs`, `cv_bridge`, OpenCV.

```bash
cd ~/ros2_ws
colcon build --packages-select watermelon_demo
source install/setup.bash
ros2 launch watermelon_demo demo.launch.py
```

The launch file starts Gazebo, bridges the camera topic, and after an 8 s
controller warm-up delay starts all four demo nodes and RViz.

**YOLO model (optional):** Place `best.pt` at `/home/lior/best.pt` and set
`use_real_model: true` in `config/demo_params.yaml`. Without the model the
pipeline runs in HSV stub mode.

**Log summary utility:**

```bash
python3 scripts/summarize_run.py run_logs/run_<timestamp>.log
python3 scripts/summarize_run.py run_logs/run_<timestamp>.log --full
```

---

## Repository Layout

```
src/watermelon_demo/
├── watermelon_demo/
│   ├── arm_controller_node.py    scan → centre → fire state machine
│   ├── detection_node.py         camera → YOLO / HSV → /detections
│   ├── laser_effect_node.py      /laser_fire → RViz beam arrow
│   └── field_manager_node.py     field plant markers, treated-weed tracking
├── launch/     demo.launch.py
├── config/     demo_params.yaml, demo_rviz.rviz
├── urdf/       ur5_with_sensors.urdf.xacro
├── worlds/     watermelon_field.sdf
└── test/       pytest suite (unit + integration, 108 tests)
docs/
├── architecture.md               node graph, topic table, TF tree
├── weed_centering_control_loop.md  state machine diagram
└── calibration_log.md            wrist_2 gain derivation from logs
scripts/
└── summarize_run.py              run-log pretty-printer
```

---

## Detection Classes

| ID | Label | Notes |
|---|---|---|
| 0 | `watermelon` | Ignored by arm controller |
| 1 | `weed_side` | Lateral weed view — targeting point adjusted to soil base |
| 2 | `weed_top` | Top-down weed view — targeting point at upper centre |

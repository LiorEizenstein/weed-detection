# System Architecture

ROS 2 (Jazzy) + Gazebo Sim 8 simulation of a UR5 arm scanning a watermelon
field, detecting weeds from a wrist-mounted camera, and firing a Tm:YLF laser
at each weed.

---

## Node graph

```
┌─────────────────────────────────────────────────────────────────────┐
│  Gazebo Sim 8                                                       │
│  ┌─────────────────────────────────────────┐                        │
│  │  UR5 arm  +  wrist camera  +  field SDF │                        │
│  └────┬──────────────────────┬─────────────┘                        │
│       │ /camera/image_raw    │ joint states / TF tree               │
│       │ (gz.msgs.Image)      │ (via ros2_control)                   │
└───────┼──────────────────────┼──────────────────────────────────────┘
        │ ros_gz_bridge        │
        ▼                      ▼
┌──────────────────┐    ┌──────────────────────────────────────────┐
│  detection_node  │    │  arm_controller_node                     │
│                  │    │                                          │
│  /camera/image   │    │  state machine: INIT → SCAN_MOVE →      │
│  → YOLO / HSV   │    │  SCANNING → WAITING_DETECTION →          │
│  → /detections   │    │  MOVE_TO_WEED → FIRE_LASER → FIRING     │
│  → /detection_   │    │                                          │
│    image         │    │  action client:                          │
└────────┬─────────┘    │  /scaled_joint_trajectory_controller/   │
         │              │    follow_joint_trajectory               │
         │ /detections  └──────────────┬───────────────────────────┘
         │ (Detection2DArray)          │ /laser_fire (Bool)
         ▼                             │
┌──────────────────┐             ┌─────┴────────────────────────────┐
│  laser_effect_   │◄────────────┤  /laser_fire                     │
│  node            │             └──────────────────────────────────┘
│                  │◄── /detections (caches weed pixel for beam aim)
│  TF lookup:      │
│  camera_link →   │             ┌──────────────────────────────────┐
│  world           │             │  field_manager_node              │
│  laser_link →    │             │                                  │
│  world           │             │  listens to /laser_fire →        │
│                  │             │  marks nearest weed as treated   │
│  → /visualization│             │  → /field_markers (MarkerArray)  │
│    _marker       │             │    red=untreated, grey=treated   │
│  (ARROW beam)    │             └──────────────────────────────────┘
└──────────────────┘

                        RViz2
                        ├── /detection_image   (annotated camera feed)
                        ├── /field_markers     (plant + weed cylinders)
                        └── /visualization_marker  (laser beam arrow)
```

---

## Topic reference

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → detection_node |
| `/detections` | `vision_msgs/Detection2DArray` | detection_node → arm_controller_node, laser_effect_node |
| `/detection_image` | `sensor_msgs/Image` | detection_node → RViz |
| `/laser_fire` | `std_msgs/Bool` | arm_controller_node → laser_effect_node, field_manager_node |
| `/field_markers` | `visualization_msgs/MarkerArray` | field_manager_node → RViz |
| `/visualization_marker` | `visualization_msgs/Marker` | laser_effect_node → RViz |
| `/scaled_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` (action) | arm_controller_node → ros2_control |

---

## TF tree (relevant frames)

```
world
└── base_link  (UR5 base, fixed)
    └── shoulder_link
        └── upper_arm_link
            └── forearm_link
                └── wrist_1_link
                    └── wrist_2_link
                        └── wrist_3_link
                            ├── tool0
                            ├── camera_link   ← image source, used for ray projection
                            └── laser_link    ← laser origin for beam visualisation
```

`camera_link` and `laser_link` are co-mounted on `tool0` via the URDF
(`ur5_with_sensors.urdf.xacro`). `robot_state_publisher` keeps all frames
up-to-date from joint states.

---

## Node responsibilities

### `detection_node`
Subscribes to `/camera/image_raw`. In production mode runs YOLO (`best.pt`,
classes 0=watermelon, 1=weed_side, 2=weed_top). In stub mode uses HSV colour
thresholds (hue 5–30, tuned for the brown Gazebo weed cylinders). Publishes
each detected bounding-box centre as a `Detection2D` with a `class_id` string
and confidence score. Also publishes an annotated image for RViz.

### `arm_controller_node`
Runs a 10 Hz state machine. Sweeps 21 pre-defined shoulder_pan poses across
±1.5 rad, dwelling at each one for up to `scan_dwell_time` (1 s) waiting for a
`/detections` message containing a weed. On detection, enters a
pixel-error-correction loop: computes normalised image error, corrects the
dominant axis (pan for X, wrist_2 for Y) by a proportional gain step, and
repeats until the weed is within 20% of image centre, then fires. Blacklists
treated pan angles within ±0.25 rad. See
[weed_centering_control_loop.md](weed_centering_control_loop.md) for the full
state machine diagram.

### `laser_effect_node`
Purely visual. On `/laser_fire True`, looks up the current `camera_link` →
`world` TF, ray-casts the cached weed pixel onto the ground plane
(z = 0.03 m), looks up `laser_link` → `world`, and publishes a red ARROW
marker from the laser origin to the computed ground point.

### `field_manager_node`
Publishes RViz cylinder/sphere markers for all plants in the field (mirroring
`watermelon_field.sdf`). On `/laser_fire True`, marks the next untreated weed
as treated (grey) in the visualisation.

---

## Key parameters (`config/demo_params.yaml`)

| Parameter | Default | Effect |
|---|---|---|
| `scan_dwell_time` | 1.0 s | How long the arm waits at each scan pose for a detection |
| `laser_fire_duration` | 1.0 s | How long `/laser_fire True` is held before moving on |
| `use_real_model` | false | Use YOLO (`true`) or HSV stub (`false`) |
| `model_path` | `/home/lior/best.pt` | Path to YOLO weights (not tracked in git) |
| `save_debug_frames` | true | Save raw/annotated/mask frames every ~1.5 s to `run_logs/frames/` |

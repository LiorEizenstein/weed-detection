# Weed Detection — UR5 Watermelon-Field Laser Demo

Final project, HUJI University.

A ROS 2 (Jazzy) + Gazebo Sim 8 simulation of a UR5 arm scanning a watermelon
field, detecting weeds from a wrist-mounted camera, and firing a visual
Tm:YLF laser beam at each weed.

## Layout

```
src/watermelon_demo/
├── worlds/      watermelon_field.sdf      Gazebo world (plants + ground)
├── urdf/        ur5_with_sensors.urdf.xacro   UR5 + camera + laser on tool0
├── watermelon_demo/   ROS 2 nodes:
│   ├── detection_node.py        camera → YOLO / HSV → /detections
│   ├── arm_controller_node.py   scan → centre → fire state machine
│   ├── laser_effect_node.py     /laser_fire → red beam marker (TF-based)
│   └── field_manager_node.py    field plant markers for RViz
├── config/      demo_params.yaml, demo_rviz.rviz
├── launch/      demo.launch.py
└── test/        pytest suite (108 tests)
```

## Build & run

```bash
cd ~/ros2_ws
colcon build --packages-select watermelon_demo
source install/setup.bash
ros2 launch watermelon_demo demo.launch.py
```

## YOLO model

The trained weights (`best.pt`) are **not** tracked in git (large binary).
Place the file at `/home/lior/best.pt` and set `use_real_model: true` in
`config/demo_params.yaml`. With `use_real_model: false` the detection node uses
an HSV colour fallback so the scan→detect→fire pipeline runs without the model.

## Classes

`0 = watermelon`, `1 = weed_side`, `2 = weed_top`

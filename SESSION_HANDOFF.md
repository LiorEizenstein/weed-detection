# Session Handoff — Real SO-101 Bring-Up (2026-06-25)

Goal: drive the physical SO-ARM101 from the existing simulation stack via
`feetech_ros2_driver` (ros2_control). Picking up after a mid-session crash.

## ✅ Done this session

- **Hardware-driver wiring is complete and builds** (colcon build at 17:25 = exit 0):
  - `so_101.ros2_control.xacro` — selects `feetech_ros2_driver/FeetechHardwareInterface`
    when `sim_backend:=hardware`; takes `usb_port` + `joint_config_file` xacro args.
  - `config/so101_feetech_joints.yaml` — per-joint Feetech STS3215 config (servo IDs 1–5,
    PID/accel). ⚠️ homing_offset / range values are still PLACEHOLDERS.
  - `launch/real.launch.py` — adds `usb_port` arg (default /dev/ttyUSB0) + `controller_manager`,
    `joint_state_broadcaster` spawner, `arm_controller` spawner, all gated on
    `use_fake_joints:=false`.
  - Verified: xacro parses with `sim_backend:=hardware`, Feetech plugin + params present in URDF,
    controllers.yaml has both controllers, `feetech_ros2_driver` installed in /opt/ros/jazzy.
- **USB passthrough working** (WSL2 via usbipd):
  - Arm = CH343 serial, Windows BUSID `1-7` (VID:PID 1a86:55d3) → **/dev/ttyACM0** in WSL.
  - Camera = USB2.0_CAM1, BUSID `1-3` → /dev/video0 + /dev/video1.
  - User is in `dialout` + `video` groups; /dev/ttyACM0 is r/w.
  - NOTE: usbipd `attach` is dropped on reboot/crash — must re-run from an **admin PowerShell**:
    `usbipd attach --wsl --busid 1-7` (arm) and `usbipd attach --wsl --busid 1-3` (camera).
    `bind` is persistent; `attach` is not.
- **Read-only servo scan tool** written: `scripts/scan_servos.py` (pings IDs 1–6, prints
  pos/volt/temp/load — never writes/moves). Needs `feetech-servo-sdk` (installed via
  `pip install --user --break-system-packages feetech-servo-sdk`).

## ⛔ Current blocker (where we stopped)

`scripts/scan_servos.py` gets **ZERO response from the servo bus** at every baud
(1M/500k/250k/115200/57600/38400) and every RTS/DTR combo — confirmed with both the SDK
and a raw pyserial probe. The CH343 bridge itself is healthy (port opens, writes succeed,
`cdc_acm → ttyACM0` bound). Silence with a working bridge = problem on the **servo side**.

Power was reportedly connected but bus still silent. **Next step is a physical check:**
1. Are any **servo LEDs lit**? (STS3215 blink on boot. No LED = servos not powered.)
2. Power into the **DC barrel jack** (servo rail), not just USB-C (which only powers board
   logic + the serial chip). Correct kit adapter voltage.
3. Board → first-servo 3-pin cable seated.
4. Power switch on, if present.

## Next steps once servos respond

1. Re-run: `python3 scripts/scan_servos.py /dev/ttyACM0 1000000` → expect 6 IDs with telemetry.
2. **Calibrate** `config/so101_feetech_joints.yaml` (fill homing_offset + range_min/max per joint;
   raw ticks 0–4095, center 2048). Required before any controlled motion.
3. Launch real stack:
   `ros2 launch real_simulation_ur5 real.launch.py use_fake_joints:=false usb_port:=/dev/ttyACM0`
   (note: ttyACM0, NOT the ttyUSB0 default).
4. Open items: nothing committed yet (all M/??); no gripper controller wired (5 arm joints only).

#!/usr/bin/env python3
"""
check_camera_params.py — validate camera_params.yaml before running on the robot.

Usage:
    python3 scripts/check_camera_params.py
    python3 scripts/check_camera_params.py --params path/to/camera_params.yaml

Prints a clear summary of what TF will be published and flags any problems.
Run this after filling in camera_params.yaml with measurements or calibration output.
"""
import os, sys, math, argparse

try:
    import yaml
except ImportError:
    print("ERROR: pyyaml not installed. Run: pip install pyyaml")
    sys.exit(1)

DEFAULT_PARAMS = os.path.join(
    os.path.dirname(__file__), '..', 'src', 'watermelon_demo', 'config', 'camera_params.yaml')

GREEN  = '\033[92m'
YELLOW = '\033[93m'
RED    = '\033[91m'
RESET  = '\033[0m'
BOLD   = '\033[1m'

def ok(msg):    print(f"  {GREEN}✓{RESET} {msg}")
def warn(msg):  print(f"  {YELLOW}⚠{RESET}  {msg}")
def error(msg): print(f"  {RED}✗{RESET} {msg}")


def check(params_path):
    print(f"\n{BOLD}Checking: {params_path}{RESET}\n")

    with open(params_path) as f:
        data = yaml.safe_load(f)

    calibrated = data.get('calibrated', False)
    mount = data.get('mount', {})
    issues = 0

    # ── Mode ─────────────────────────────────────────────────────────────────
    mode = "easy_handeye2 (quaternion)" if calibrated else "manual measurement (RPY)"
    print(f"{BOLD}Mode:{RESET} {mode}")
    print()

    # ── Translation ───────────────────────────────────────────────────────────
    print(f"{BOLD}Translation (tool0 → camera_link):{RESET}")
    x = float(mount.get('x', 0))
    y = float(mount.get('y', 0))
    z = float(mount.get('z', 0))

    print(f"  x = {x:+.4f} m  (lateral)")
    print(f"  y = {y:+.4f} m  (vertical)")
    print(f"  z = {z:+.4f} m  (forward/depth)")

    if z == 0.0:
        error("z is 0 — measure the flange-to-lens distance (typically 0.04–0.08 m)")
        issues += 1
    elif z < 0.01 or z > 0.25:
        warn(f"z={z:.3f}m is outside typical range [0.01, 0.25] m — double-check")
        issues += 1
    else:
        ok(f"z looks reasonable for a wrist-mounted D435")

    if abs(x) > 0.15:
        warn(f"x={x:.3f}m is large — verify lateral offset")
        issues += 1
    if abs(y) > 0.15:
        warn(f"y={y:.3f}m is large — verify vertical offset")
        issues += 1

    print()

    # ── Rotation ─────────────────────────────────────────────────────────────
    if calibrated:
        print(f"{BOLD}Rotation (quaternion from easy_handeye2):{RESET}")
        qx = float(mount.get('qx', 0))
        qy = float(mount.get('qy', 0))
        qz = float(mount.get('qz', 0))
        qw = float(mount.get('qw', 1))

        print(f"  qx = {qx:+.6f}")
        print(f"  qy = {qy:+.6f}")
        print(f"  qz = {qz:+.6f}")
        print(f"  qw = {qw:+.6f}")

        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if abs(norm - 1.0) > 1e-4:
            error(f"Quaternion norm={norm:.6f} — not unit length, check for copy-paste errors")
            issues += 1
        else:
            ok(f"Quaternion is unit length (norm={norm:.8f})")

        is_identity = abs(qx) < 1e-9 and abs(qy) < 1e-9 and abs(qz) < 1e-9 and abs(qw - 1) < 1e-9
        if is_identity:
            error("Quaternion is identity (0,0,0,1) but calibrated=true — paste actual values")
            issues += 1

        if abs(qw) < 1e-6:
            error("qw≈0 means 180° rotation — physically impossible for a wrist mount")
            issues += 1

        # Convert to RPY for human readability
        roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
        pitch = math.asin(max(-1, min(1, 2*(qw*qy - qz*qx))))
        yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        print(f"\n  Equivalent RPY: roll={math.degrees(roll):.1f}° "
              f"pitch={math.degrees(pitch):.1f}° yaw={math.degrees(yaw):.1f}°")

    else:
        print(f"{BOLD}Rotation (manual RPY):{RESET}")
        roll  = float(mount.get('roll', 0))
        pitch = float(mount.get('pitch', 0))
        yaw   = float(mount.get('yaw', 0))

        print(f"  roll  = {roll:+.4f} rad  ({math.degrees(roll):+.1f}°)")
        print(f"  pitch = {pitch:+.4f} rad  ({math.degrees(pitch):+.1f}°)")
        print(f"  yaw   = {yaw:+.4f} rad  ({math.degrees(yaw):+.1f}°)")

        for name, val in [('roll', roll), ('pitch', pitch), ('yaw', yaw)]:
            if abs(val) > math.pi:
                error(f"{name}={val:.3f} rad is outside ±π — use radians, not degrees")
                issues += 1

        if roll == 0 and pitch == 0 and yaw == 0:
            warn("All RPY angles are 0 — OK if camera is mounted straight, "
                 "otherwise fill in the tilt angle")

    print()

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"{BOLD}Static TF that will be published:{RESET}")
    print(f"  tool0 → camera_link")
    print(f"  translation: [{x:.4f}, {y:.4f}, {z:.4f}] m")

    if issues == 0:
        print(f"\n{GREEN}{BOLD}✓ All checks passed — safe to launch demo_real.launch.py{RESET}\n")
    else:
        print(f"\n{RED}{BOLD}✗ {issues} issue(s) found — fix camera_params.yaml before launching{RESET}\n")
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Validate camera_params.yaml')
    parser.add_argument('--params', default=DEFAULT_PARAMS,
                        help='Path to camera_params.yaml')
    args = parser.parse_args()
    check(args.params)

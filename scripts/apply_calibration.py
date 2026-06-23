#!/usr/bin/env python3
"""
Read easy_handeye2 calibration output and write it into camera_params.yaml.

Usage:
    python3 scripts/apply_calibration.py <calibration.calib.yaml>

The calibration file is written by easy_handeye2 to:
    ~/.ros/easy_handeye2/<calibration_name>.calib.yaml
"""
import sys
import yaml
from pathlib import Path

def main():
    if len(sys.argv) != 2:
        calib_files = list(Path.home().glob('.ros/easy_handeye2/*.calib.yaml'))
        if calib_files:
            calib_path = sorted(calib_files)[-1]
            print(f"No file given — using most recent: {calib_path}")
        else:
            print("Usage: python3 scripts/apply_calibration.py <calibration.calib.yaml>")
            print("No calibration files found in ~/.ros/easy_handeye2/")
            sys.exit(1)
    else:
        calib_path = Path(sys.argv[1])

    if not calib_path.exists():
        print(f"ERROR: File not found: {calib_path}")
        sys.exit(1)

    with open(calib_path) as f:
        calib = yaml.safe_load(f)

    transform = calib.get('transformation', calib)
    tx = float(transform['x'])
    ty = float(transform['y'])
    tz = float(transform['z'])
    qx = float(transform['qx'])
    qy = float(transform['qy'])
    qz = float(transform['qz'])
    qw = float(transform['qw'])

    params_path = Path(__file__).parent.parent / 'src/watermelon_demo/config/camera_params.yaml'
    with open(params_path) as f:
        params = yaml.safe_load(f)

    params['calibrated'] = True
    params['mount']['x']  = round(tx, 6)
    params['mount']['y']  = round(ty, 6)
    params['mount']['z']  = round(tz, 6)
    params['mount']['qx'] = round(qx, 6)
    params['mount']['qy'] = round(qy, 6)
    params['mount']['qz'] = round(qz, 6)
    params['mount']['qw'] = round(qw, 6)

    with open(params_path, 'w') as f:
        yaml.dump(params, f, default_flow_style=False, sort_keys=False)

    print(f"Written to {params_path}:")
    print(f"  x={tx:.6f}  y={ty:.6f}  z={tz:.6f}")
    print(f"  qx={qx:.6f}  qy={qy:.6f}  qz={qz:.6f}  qw={qw:.6f}")
    print(f"  calibrated: true")
    print()
    print("Verify with:")
    print("  python3 scripts/check_camera_params.py")

if __name__ == '__main__':
    main()

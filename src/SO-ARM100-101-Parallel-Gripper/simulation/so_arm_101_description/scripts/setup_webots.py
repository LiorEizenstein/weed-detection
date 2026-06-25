#!/usr/bin/env python3
"""Convert SO-ARM-101 URDF to a Webots world with the robot embedded.

Prerequisites (install inside a ROS 2 environment):
    pip install urdf2webots

Usage:
    # From the package root, with ROS 2 workspace sourced:
    python3 scripts/setup_webots.py

    # Or specify a custom output directory:
    python3 scripts/setup_webots.py --output-dir /path/to/output

Outputs:
    worlds/webots_sim.wbt   — World file with the robot embedded,
                              controller set to "<extern>" for webots_ros2_driver
"""
import argparse
import os
import subprocess
import sys
import tempfile

_WEBOTS_BASE = (
    "https://raw.githubusercontent.com/cyberbotics/webots"
    "/R2023b/projects/objects"
)
_BG = _WEBOTS_BASE + "/backgrounds/protos"
_FLOOR = _WEBOTS_BASE + "/floors/protos"

WORLD_HEADER = (
    "#VRML_SIM R2023b utf8\n"
    "\n"
    f'EXTERNPROTO "{_BG}/TexturedBackground.proto"\n'
    f'EXTERNPROTO "{_BG}/TexturedBackgroundLight.proto"\n'
    f'EXTERNPROTO "{_FLOOR}/RectangleArena.proto"\n'
    "\n"
    "WorldInfo {\n"
    "  basicTimeStep 16\n"
    '  title "SO-ARM-101"\n'
    "  contactProperties [\n"
    "    ContactProperties {\n"
    "      maxContactJoints 4\n"
    "    }\n"
    "  ]\n"
    "}\n"
    "Viewpoint {\n"
    "  orientation -0.25 -0.66 -0.71 0.74\n"
    "  position -0.45 0.65 0.8\n"
    "}\n"
    "TexturedBackground {\n"
    "}\n"
    "TexturedBackgroundLight {\n"
    "}\n"
    "RectangleArena {\n"
    "  floorSize 2 2\n"
    "}\n"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output-dir',
        default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        help='Package root directory (default: auto-detected)',
    )
    args = parser.parse_args()

    pkg_dir = args.output_dir
    worlds_dir = os.path.join(pkg_dir, 'worlds')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'so_101.urdf.xacro')

    if not os.path.isfile(xacro_file):
        print(f"Error: xacro file not found: {xacro_file}", file=sys.stderr)
        return 1

    # Step 1: Generate URDF from xacro
    print("Running xacro (sim_backend:=webots) ...")
    result = subprocess.run(
        ['xacro', xacro_file, 'sim_backend:=webots'],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"xacro failed:\n{result.stderr}", file=sys.stderr)
        return 1

    with tempfile.NamedTemporaryFile(
        suffix='.urdf', delete=False, mode='w', encoding='utf-8',
    ) as f:
        f.write(result.stdout)
        urdf_path = f.name

    # Step 2: Convert URDF to Webots Robot node via urdf2webots
    print("Converting URDF to Webots Robot node ...")
    try:
        from urdf2webots.importer import convertUrdfFile
        robot_node = convertUrdfFile(
            input=urdf_path,
            robotName='so_101',
        )
    except ImportError:
        print(
            "Error: urdf2webots is not installed. "
            "Install it with: pip install urdf2webots",
            file=sys.stderr,
        )
        return 1
    except Exception as e:
        print(f"urdf2webots conversion failed: {e}", file=sys.stderr)
        return 1
    finally:
        os.unlink(urdf_path)

    if not robot_node:
        print("Error: urdf2webots returned empty result", file=sys.stderr)
        return 1

    # Step 3: Patch Robot node — add controller "<extern>" and ensure name
    # This is required for WebotsController to connect via IPC.
    if 'controller' not in robot_node:
        robot_node = robot_node.replace(
            'Robot {',
            'Robot {\n  controller "<extern>"',
            1,
        )
    if 'name "so_101"' not in robot_node:
        robot_node = robot_node.replace(
            'Robot {',
            'Robot {\n  name "so_101"',
            1,
        )

    # Step 4: Generate world file with robot embedded
    world_path = os.path.join(worlds_dir, 'webots_sim.wbt')

    print(f"Writing world file: {world_path}")
    with open(world_path, 'w', newline='\n', encoding='utf-8') as f:
        f.write(WORLD_HEADER)
        f.write(robot_node)
        f.write('\n')

    print("Done!")
    print(f"  World: {world_path}")
    print()
    print("To launch Webots:")
    print("  ros2 launch so_arm_101_description sim.launch.py sim:=webots")
    return 0


if __name__ == '__main__':
    sys.exit(main())

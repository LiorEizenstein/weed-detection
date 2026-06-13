#!/usr/bin/env python3
"""
YAML compatibility test for all xacro/URDF files in this package.

ROS2's launch framework passes robot_description as a YAML parameter.
Any XML comment containing "key: value" patterns (e.g. "xyz: ...", "rpy: ...")
will trip the YAML parser and crash the launch. This test catches that early.

Run at every step that creates or modifies a xacro/URDF file:
  python3 ~/ros2_ws/src/watermelon_demo/test_yaml_compat.py
"""

import subprocess
import sys
import yaml
import re
from pathlib import Path

PACKAGE_DIR = Path(__file__).parent
URDF_DIR = PACKAGE_DIR / "urdf"

XACRO_ARGS = {
    "ur5_with_sensors.urdf.xacro": ["ur_type:=ur5"],
}

COMMENT_PATTERN = re.compile(r"<!--.*?:\s+.*?-->", re.DOTALL)

def check_source_comments(xacro_path: Path) -> list[str]:
    """Detect key: value patterns inside XML comments in the source file."""
    issues = []
    content = xacro_path.read_text()
    for match in COMMENT_PATTERN.finditer(content):
        comment = match.group()
        # find all key: value occurrences inside this comment
        kv = re.findall(r"\b\w+:\s+\S", comment)
        if kv:
            line_no = content[:match.start()].count("\n") + 1
            issues.append(f"  Line ~{line_no}: comment contains key: value pattern — {kv}")
    return issues

def check_xacro_yaml(xacro_path: Path, extra_args: list[str]) -> list[str]:
    """Run xacro and verify the output passes yaml.safe_load."""
    issues = []
    result = subprocess.run(
        ["xacro", str(xacro_path)] + extra_args,
        capture_output=True, text=True
    )
    if result.returncode != 0:
        issues.append(f"  xacro failed:\n{result.stderr[:500]}")
        return issues

    try:
        yaml.safe_load(result.stdout)
    except yaml.scanner.ScannerError as e:
        lines = result.stdout.split("\n")
        line_no = e.problem_mark.line
        bad_line = lines[line_no] if line_no < len(lines) else "(unknown)"
        issues.append(
            f"  yaml.safe_load failed at line {line_no + 1}: {bad_line.strip()}\n"
            f"  Cause: {e.problem}"
        )
    return issues

def main():
    print("=" * 60)
    print("YAML Compatibility Test")
    print("=" * 60)

    xacro_files = list(URDF_DIR.glob("*.xacro"))
    if not xacro_files:
        print("No xacro files found in urdf/ — skipping.")
        sys.exit(0)

    all_passed = True

    for xacro_file in sorted(xacro_files):
        print(f"\nChecking: {xacro_file.name}")
        extra_args = XACRO_ARGS.get(xacro_file.name, [])

        # 1. Source-level comment check
        comment_issues = check_source_comments(xacro_file)
        if comment_issues:
            print("  [FAIL] Dangerous comment patterns found:")
            for issue in comment_issues:
                print(issue)
            all_passed = False
        else:
            print("  [PASS] No key: value patterns in comments")

        # 2. Runtime YAML parse check
        yaml_issues = check_xacro_yaml(xacro_file, extra_args)
        if yaml_issues:
            print("  [FAIL] yaml.safe_load failed on xacro output:")
            for issue in yaml_issues:
                print(issue)
            all_passed = False
        else:
            print("  [PASS] xacro output passes yaml.safe_load")

    print("\n" + "=" * 60)
    if all_passed:
        print("ALL TESTS PASSED")
    else:
        print("SOME TESTS FAILED — fix before proceeding")
    print("=" * 60)

    sys.exit(0 if all_passed else 1)

if __name__ == "__main__":
    main()

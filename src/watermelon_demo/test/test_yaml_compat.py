"""
YAML compatibility tests for all xacro/URDF files.

ROS2's launch framework passes robot_description as a YAML parameter.
Any XML comment containing "key: value" patterns will trip the YAML
parser and crash the launch silently or with a confusing error.

Run with:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_yaml_compat.py -v
"""

import re
import subprocess
import yaml
import pytest
from pathlib import Path

URDF_DIR = Path(__file__).parent.parent / "urdf"

# Args to pass when processing each xacro file
XACRO_ARGS = {
    "ur5_with_sensors.urdf.xacro": ["ur_type:=ur5"],
}

COMMENT_PATTERN = re.compile(r"<!--.*?-->", re.DOTALL)
KEY_VALUE_PATTERN = re.compile(r"\b\w+:\s+\S")


def get_xacro_files():
    """Collect all xacro files from the urdf directory."""
    return list(URDF_DIR.glob("*.xacro"))


def pytest_generate_tests(metafunc):
    """Parametrize tests over all discovered xacro files."""
    if "xacro_file" in metafunc.fixturenames:
        files = get_xacro_files()
        metafunc.parametrize("xacro_file", files, ids=[f.name for f in files])


class TestYamlCompatibility:

    def test_no_key_value_in_comments(self, xacro_file):
        """XML comments must not contain 'key: value' patterns.

        Such patterns cause yaml.safe_load to raise ScannerError when
        ROS2's launch framework tries to set robot_description as a parameter.
        """
        content = xacro_file.read_text()
        violations = []

        for match in COMMENT_PATTERN.finditer(content):
            comment = match.group()
            hits = KEY_VALUE_PATTERN.findall(comment)
            if hits:
                line_no = content[: match.start()].count("\n") + 1
                violations.append(f"Line ~{line_no}: {hits} in comment: {comment[:80]!r}")

        assert not violations, (
            f"{xacro_file.name} has key: value patterns in XML comments "
            f"that will break ROS2 YAML parameter parsing:\n"
            + "\n".join(violations)
        )

    def test_xacro_generates_valid_xml(self, xacro_file):
        """xacro must exit 0 and produce non-empty output."""
        extra_args = XACRO_ARGS.get(xacro_file.name, [])
        result = subprocess.run(
            ["xacro", str(xacro_file)] + extra_args,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, (
            f"xacro failed for {xacro_file.name}:\n{result.stderr[:1000]}"
        )
        assert result.stdout.strip(), f"xacro produced empty output for {xacro_file.name}"
        assert "<robot" in result.stdout, (
            f"xacro output for {xacro_file.name} does not contain a <robot> element"
        )

    def test_xacro_output_passes_yaml_load(self, xacro_file):
        """xacro output must be parseable by yaml.safe_load.

        ROS2 launch passes robot_description through yaml.safe_load before
        setting it as a node parameter. If this fails, the launch crashes.
        """
        extra_args = XACRO_ARGS.get(xacro_file.name, [])
        result = subprocess.run(
            ["xacro", str(xacro_file)] + extra_args,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            pytest.skip(f"xacro failed — covered by test_xacro_generates_valid_xml")

        lines = result.stdout.split("\n")
        try:
            yaml.safe_load(result.stdout)
        except yaml.scanner.ScannerError as e:
            line_no = e.problem_mark.line
            bad_line = lines[line_no] if line_no < len(lines) else "(unknown)"
            pytest.fail(
                f"yaml.safe_load failed on {xacro_file.name} output at line {line_no + 1}:\n"
                f"  {bad_line.strip()}\n"
                f"  Problem: {e.problem}\n"
                f"  Fix: remove 'key: value' patterns from XML comments near that line."
            )

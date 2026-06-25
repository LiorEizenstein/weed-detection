"""Test that MuJoCo can load the robot URDF."""
import os
import re
import xml.etree.ElementTree as ET

import pytest

BASE_DIR = os.path.join(os.path.dirname(__file__), '..')


def _build_plain_urdf() -> str:
    """Expand xacro into plain URDF for MuJoCo (without ROS2 xacro tool)."""
    # Parse main xacro
    main_tree = ET.parse(os.path.join(BASE_DIR, 'urdf', 'so_101.urdf.xacro'))
    root = main_tree.getroot()

    # Remove xacro namespace elements (arg, include)
    ns = 'http://www.ros.org/wiki/xacro'
    to_remove = []
    for el in root:
        tag = el.tag
        if isinstance(tag, str) and (ns in tag or tag.startswith('{' + ns)):
            to_remove.append(el)
        elif isinstance(tag, str) and tag.startswith('xacro:'):
            to_remove.append(el)
    for el in to_remove:
        root.remove(el)

    # Inline materials
    mat_tree = ET.parse(os.path.join(BASE_DIR, 'urdf', 'materials.xacro'))
    mat_root = mat_tree.getroot()
    for mat in mat_root.findall('material'):
        root.insert(0, mat)

    # Replace package:// paths with absolute filesystem paths (no file:// URI)
    urdf_str = ET.tostring(root, encoding='unicode')
    abs_base = os.path.abspath(BASE_DIR).replace('\\', '/')
    urdf_str = urdf_str.replace('package://so_arm_101_description/', f'{abs_base}/')

    # Replace $(find ...) in mujoco compiler tag with absolute paths
    urdf_str = re.sub(
        r'\$\(find so_arm_101_description\)',
        abs_base,
        urdf_str,
    )

    # Clean up xacro namespace declarations
    urdf_str = re.sub(r'\s*xmlns:xacro="[^"]*"', '', urdf_str)

    return urdf_str


@pytest.fixture(scope='module')
def mujoco_model():
    import mujoco

    urdf_str = _build_plain_urdf()
    tmp_path = os.path.join(BASE_DIR, 'test', '_tmp_test.urdf')
    try:
        with open(tmp_path, 'w') as f:
            f.write(urdf_str)
        model = mujoco.MjModel.from_xml_path(tmp_path)
        yield model
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


def test_mujoco_loads(mujoco_model):
    assert mujoco_model is not None


def test_mujoco_body_count(mujoco_model):
    # world/base_link (merged) + 5 arm links + 2 clamps = 8
    assert mujoco_model.nbody == 8, f"Expected 8 bodies, got {mujoco_model.nbody}"


def test_mujoco_joint_count(mujoco_model):
    assert mujoco_model.njnt == 7, f"Expected 7 joints, got {mujoco_model.njnt}"


def test_mujoco_dof_count(mujoco_model):
    assert mujoco_model.nv == 7, f"Expected 7 DOF, got {mujoco_model.nv}"


def test_mujoco_can_step(mujoco_model):
    import mujoco
    data = mujoco.MjData(mujoco_model)
    mujoco.mj_step(mujoco_model, data)
    # Should not crash; time should advance
    assert data.time > 0

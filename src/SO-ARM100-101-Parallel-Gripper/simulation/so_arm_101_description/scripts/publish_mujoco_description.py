#!/usr/bin/env python3
"""Publish MJCF model on /mujoco_robot_description for mujoco_ros2_control.

The MujocoSystemInterface plugin expects an XML model (URDF or MJCF) on a
latched topic.  This node:
1. Reads the ``robot_description`` parameter (URDF)
2. Loads it into MuJoCo (which can parse URDF natively)
3. Adds scene elements (lights, floor), position actuators, and fixes
   mesh paths to use visual meshes instead of collision hulls
4. Exports the result as MJCF XML and publishes it
"""
import os
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def urdf_to_mjcf_with_actuators(urdf_xml: str) -> str:
    """Convert URDF to MJCF with actuators, lights, floor, and visual meshes."""
    # Parse URDF to extract non-fixed joint names and limits
    urdf_root = ET.fromstring(urdf_xml)
    joints = []
    for j in urdf_root.findall('.//joint'):
        jtype = j.attrib.get('type', 'fixed')
        if jtype == 'fixed':
            continue
        name = j.attrib['name']
        limit = j.find('limit')
        lo = float(limit.attrib.get('lower', '-3.14159'))
        hi = float(limit.attrib.get('upper', '3.14159'))
        joints.append((name, jtype, lo, hi))

    # Load URDF into MuJoCo and export as MJCF
    model = mujoco.MjModel.from_xml_string(urdf_xml)
    with tempfile.NamedTemporaryFile(
        suffix='.xml', delete=False, mode='w',
    ) as f:
        tmp_path = f.name
    try:
        mujoco.mj_saveLastXML(tmp_path, model)
        with open(tmp_path, encoding='utf-8') as f:
            mjcf_xml = f.read()
    finally:
        os.unlink(tmp_path)

    mjcf_root = ET.fromstring(mjcf_xml)

    # --- Fix meshdir to use visual meshes instead of collision ---
    compiler = mjcf_root.find('compiler')
    if compiler is not None:
        meshdir = compiler.get('meshdir', '')
        if 'collision' in meshdir:
            compiler.set('meshdir', meshdir.replace('collision', 'visual'))

    # --- Add scene: lights + floor ---
    worldbody = mjcf_root.find('worldbody')

    # Add light
    ET.SubElement(worldbody, 'light', {
        'name': 'top_light',
        'pos': '0 0 1.5',
        'dir': '0 0 -1',
        'diffuse': '0.8 0.8 0.8',
        'specular': '0.3 0.3 0.3',
        'directional': 'true',
    })
    ET.SubElement(worldbody, 'light', {
        'name': 'front_light',
        'pos': '0.5 -0.5 1.0',
        'dir': '-0.3 0.3 -0.5',
        'diffuse': '0.4 0.4 0.4',
    })

    # Add floor
    ET.SubElement(worldbody, 'geom', {
        'name': 'floor',
        'type': 'plane',
        'size': '1 1 0.01',
        'rgba': '0.3 0.35 0.4 1',
        'contype': '1',
        'conaffinity': '1',
    })

    # --- Increase joint damping and enable gravity compensation ---
    for joint_elem in mjcf_root.iter('joint'):
        if joint_elem.get('type', 'hinge') in ('hinge', 'slide'):
            joint_elem.set('damping', '10.0')

    # Enable gravity compensation on all bodies
    for body_elem in mjcf_root.iter('body'):
        body_elem.set('gravcomp', '1')

    # --- Add position actuators ---
    actuator_elem = mjcf_root.find('actuator')
    if actuator_elem is None:
        actuator_elem = ET.SubElement(mjcf_root, 'actuator')

    for name, jtype, lo, hi in joints:
        ET.SubElement(actuator_elem, 'position', {
            'name': name,
            'joint': name,
            'kp': '5',
            'kv': '20',
            'ctrlrange': f'{lo} {hi}',
            'ctrllimited': 'true',
        })

    return ET.tostring(mjcf_root, encoding='unicode', xml_declaration=True)


class MujocoDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('mujoco_description_publisher')
        self.declare_parameter('robot_description', '')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._pub = self.create_publisher(
            String, '/mujoco_robot_description', qos,
        )

        desc = self.get_parameter('robot_description').value
        if not desc:
            self.get_logger().error('robot_description parameter is empty!')
            return

        try:
            mjcf = urdf_to_mjcf_with_actuators(desc)
        except Exception as e:
            self.get_logger().error(f'Failed to convert URDF to MJCF: {e}')
            return

        msg = String()
        msg.data = mjcf
        self._pub.publish(msg)
        self.get_logger().info(
            f'Published MJCF ({len(mjcf)} bytes) on '
            '/mujoco_robot_description',
        )


def main() -> None:
    rclpy.init()
    node = MujocoDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

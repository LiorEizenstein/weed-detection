"""
joint_state_restamper — republishes /joint_states with current wall time.

Gazebo's joint_state_broadcaster sets header.stamp = sim time (stuck at 0 on
WSL2), so robot_state_publisher timestamps all TF transforms at t=0.  RViz
(wall time) can never find those transforms and reports "no transform" for
every movable link.

This node copies each JointState message verbatim but replaces the header
stamp with now() (wall time), then publishes to /joint_states_stamped.
robot_state_publisher is remapped to subscribe to that topic instead.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRestamper(Node):

    def __init__(self):
        super().__init__('joint_state_restamper')
        self._pub = self.create_publisher(JointState, '/joint_states_stamped', 10)
        self.create_subscription(JointState, '/joint_states', self._cb, 10)

    def _cb(self, msg: JointState):
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointStateRestamper())

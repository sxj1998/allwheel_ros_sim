#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        self.declare_parameter('robot_description', '')
        self.description = self.get_parameter('robot_description').value

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(String, 'robot_description', qos)
        self.timer = self.create_timer(1.0, self.on_timer)

        if not self.description:
            self.get_logger().warn('robot_description is empty; check URDF/xacro.')

    def on_timer(self):
        if not self.description:
            self.description = self.get_parameter('robot_description').value
            if not self.description:
                return
        self.publisher.publish(String(data=self.description))


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

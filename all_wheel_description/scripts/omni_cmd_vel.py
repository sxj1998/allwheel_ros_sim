#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OmniCmdVel(Node):
    def __init__(self):
        super().__init__('omni_cmd_vel')

        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('base_radius', 0.099)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.25)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('command_topic', '/wheel_velocity_controller/commands')
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.base_radius = float(self.get_parameter('base_radius').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.cmd_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.latest_wheel_vel = None
        self.last_joint_positions = {}
        self.last_joint_time = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

    def on_cmd_vel(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_joint_state(self, msg):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        required = ['front_wheel_joint', 'right_wheel_joint', 'left_wheel_joint']
        if not all(name in name_to_idx for name in required):
            return

        velocities = None
        if msg.velocity and len(msg.velocity) >= len(msg.name):
            velocities = [msg.velocity[name_to_idx[n]] for n in required]
        elif msg.position:
            now = self.get_clock().now()
            if self.last_joint_time is not None:
                dt = (now - self.last_joint_time).nanoseconds * 1e-9
                if dt > 0.0:
                    velocities = []
                    for name in required:
                        idx = name_to_idx[name]
                        pos = msg.position[idx]
                        last_pos = self.last_joint_positions.get(name, pos)
                        velocities.append((pos - last_pos) / dt)
            self.last_joint_positions = {name: msg.position[name_to_idx[name]] for name in required}
            self.last_joint_time = now

        if velocities is not None:
            self.latest_wheel_vel = velocities

    def compute_wheel_speeds(self, vx, vy, wz):
        if self.wheel_radius <= 0.0:
            return [0.0, 0.0, 0.0]

        inv_r = 1.0 / self.wheel_radius
        half_sqrt3 = math.sqrt(3.0) / 2.0

        w0 = inv_r * (vy + self.base_radius * wz)
        w1 = inv_r * (-0.5 * vy + half_sqrt3 * vx + self.base_radius * wz)
        w2 = inv_r * (-0.5 * vy - half_sqrt3 * vx + self.base_radius * wz)
        return [w0, w1, w2]

    def compute_base_twist(self, wheel_vel):
        if self.wheel_radius <= 0.0 or self.base_radius <= 0.0:
            return 0.0, 0.0, 0.0

        w0, w1, w2 = wheel_vel
        vx = (self.wheel_radius / math.sqrt(3.0)) * (w1 - w2)
        vy = (self.wheel_radius / 3.0) * (2.0 * w0 - w1 - w2)
        wz = (self.wheel_radius / (3.0 * self.base_radius)) * (w0 + w1 + w2)
        return vx, vy, wz

    def on_timer(self):
        now = self.get_clock().now()
        stale = (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_vel_timeout

        cmd = Twist()
        if not stale:
            cmd = self.last_cmd

        wheel_vel = self.compute_wheel_speeds(cmd.linear.x, cmd.linear.y, cmd.angular.z)
        self.cmd_pub.publish(Float64MultiArray(data=wheel_vel))

        if self.latest_wheel_vel is None:
            return

        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_odom_time = now

        vx, vy, wz = self.compute_base_twist(self.latest_wheel_vel)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw += wz * dt

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = OmniCmdVel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

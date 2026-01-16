#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from geometry_msgs.msg import Twist


MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 0.0, 1.0),
    'd': (0.0, 0.0, -1.0),
    'q': (1.0, 0.0, 1.0),
    'e': (1.0, 0.0, -1.0),
    'z': (-1.0, 0.0, 1.0),
    'c': (-1.0, 0.0, -1.0),
}


def get_key(timeout, settings, input_stream):
    tty.setraw(input_stream.fileno())
    rlist, _, _ = select.select([input_stream], [], [], timeout)
    if rlist:
        key = input_stream.read(1)
    else:
        key = ''
    termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)
    return key


def print_instructions():
    msg = (
        "Keyboard teleop (focus this terminal)\n"
        "w/s: forward/back\n"
        "a/d: rotate left/right\n"
        "q/e: forward + rotate\n"
        "z/c: back + rotate\n"
        "space or x: stop\n"
        "CTRL-C: quit\n"
    )
    print(msg)


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    linear_speed = 0.3
    angular_speed = 0.8

    input_stream = sys.stdin
    close_stream = False
    if not input_stream.isatty():
        try:
            input_stream = open('/dev/tty')
            close_stream = True
        except OSError:
            print('keyboard_teleop requires a TTY. Run from a terminal.', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return

    try:
        settings = termios.tcgetattr(input_stream)
    except termios.error:
        if close_stream:
            input_stream.close()
        print('keyboard_teleop requires a TTY. Run from a terminal.', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return
    print_instructions()

    last_moving = False
    try:
        while rclpy.ok():
            key = get_key(0.1, settings, input_stream)
            twist = Twist()

            if key in MOVE_BINDINGS:
                x, _, z = MOVE_BINDINGS[key]
                twist.linear.x = x * linear_speed
                twist.angular.z = z * angular_speed
                pub.publish(twist)
                last_moving = True
            elif key in [' ', 'x']:
                pub.publish(twist)
                last_moving = False
            elif key == '\x03':
                break
            else:
                if last_moving:
                    pub.publish(twist)
                    last_moving = False
    finally:
        termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)
        if close_stream:
            input_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

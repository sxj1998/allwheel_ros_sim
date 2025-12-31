from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='all_wheel_description',
        executable='keyboard_teleop.py',
        name='keyboard_teleop',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([teleop_node])

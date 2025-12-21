from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_package = LaunchConfiguration('urdf_package')
    urdf_name = LaunchConfiguration('urdf_name')

    declare_urdf_package = DeclareLaunchArgument(
        'urdf_package',
        default_value='all_wheel_description',
        description='Package that contains the URDF file',
    )
    declare_urdf_name = DeclareLaunchArgument(
        'urdf_name',
        default_value='all_wheel.urdf',
        description='URDF file name under the urdf directory',
    )

    urdf_model_path = PathJoinSubstitution([
        FindPackageShare(package=urdf_package),
        'urdf',
        urdf_name,
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_urdf_package)
    ld.add_action(declare_urdf_name)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld

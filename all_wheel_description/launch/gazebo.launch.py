import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'all_wheel_description'
    pkg_share = get_package_share_directory(package_name)

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'world', 'hourse.world'),
        description='Gazebo Sim world file',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true',
    )
    declare_gz_version = DeclareLaunchArgument(
        'gz_version',
        default_value='6',
        description='Gazebo Sim major version (ROS 2 Humble matches Ignition Gazebo 6)',
    )

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_version = LaunchConfiguration('gz_version')

    resource_path_list = [
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        str(Path(pkg_share).parent.resolve()),
        pkg_share,
        os.path.join(pkg_share, 'world'),
        os.path.join(pkg_share, 'models'),
        os.path.join(pkg_share, 'meshes'),
        '/usr/share/gazebo-11',
    ]
    resource_paths = os.pathsep.join([p for p in resource_path_list if p])
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_paths,
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_paths,
    )

    urdf_file = os.path.join(pkg_share, 'urdf', 'all_wheel.urdf')
    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    gz_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_launch_path]),
        launch_arguments={
            'gz_args': [world, ' -r -v 4'],
            'gz_version': gz_version,
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'all_wheel', '-z', '0.05'],
        output='screen',
    )

    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
    )

    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'wheel_velocity_controller'],
        output='screen',
    )

    omni_cmd_vel = Node(
        package=package_name,
        executable='omni_cmd_vel.py',
        name='omni_cmd_vel',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_gz_version)
    ld.add_action(set_gz_resource_path)
    ld.add_action(set_ign_resource_path)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(ros_gz_bridge)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_controllers)
    ld.add_action(omni_cmd_vel)

    return ld

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_pkg = FindPackageShare('all_wheel_navigation')
    desc_pkg = FindPackageShare('all_wheel_description')
    nav2_pkg = FindPackageShare('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    launch_sim = LaunchConfiguration('launch_sim')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true.',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack.',
    )
    declare_launch_sim = DeclareLaunchArgument(
        'launch_sim',
        default_value='true',
        description='Launch Gazebo simulation with the robot.',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([nav_pkg, 'config', 'nav2_params.yaml']),
        description='Full path to the nav2 parameters file.',
    )
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to the map YAML file for localization.',
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desc_pkg, 'launch', 'gazebo.launch.py'])
        ),
        condition=IfCondition(launch_sim),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_pkg, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'slam': 'False',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map': map_yaml,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_launch_sim,
        declare_params_file,
        declare_map_yaml,
        gazebo_launch,
        nav2_launch,
    ])

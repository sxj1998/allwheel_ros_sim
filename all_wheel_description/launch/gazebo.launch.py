from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        ])
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_model_path, '-entity', 'all_wheel'],
        output='screen',
    )

    return LaunchDescription([
        declare_urdf_package,
        declare_urdf_name,
        gz_launch,
        rsp_node,
        spawn_entity,
    ])

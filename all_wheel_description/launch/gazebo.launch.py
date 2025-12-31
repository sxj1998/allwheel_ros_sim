import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 基本标识
    robot_name_in_model = 'all_wheel'
    package_name = 'all_wheel_description'
    urdf_name = "all_wheel.urdf"

    # 资源路径
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)
    gazebo_world_path = os.path.join(pkg_share, 'world', 'bot.world')

    # Gazebo 模型路径设置（保证 model:// 可解析）
    model_path_root = os.path.join(pkg_share, 'models')
    map_model_path = os.path.join(pkg_share, 'world', 'map')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join([
            p for p in [model_path_root, map_model_path, os.environ.get('GAZEBO_MODEL_PATH', '')] if p
        ])
    )

    # 禁用在线模型库（避免网络阻塞）
    disable_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    # 启动 Gazebo Classic（加载 ROS 插件与世界）
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path
        ],
        output='screen'
    )

    # 机器人插入（延长 timeout，避免启动慢）
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-file', urdf_model_path,
            '-timeout', '120'
        ],
        output='screen'
    )

    # Gazebo 启动后再延迟几秒执行 spawn
    delayed_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=start_gazebo_cmd,
            on_start=[TimerAction(period=1.0, actions=[spawn_entity_cmd])]
        )
    )

    # ROS 2 节点：TF、关节状态、可视化
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        output='screen',
    )
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # 组装 LaunchDescription
    ld = LaunchDescription()
    ld.add_action(set_gazebo_model_path)
    ld.add_action(disable_model_db)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(delayed_spawn)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(joint_state_publisher_cmd)

    return ld

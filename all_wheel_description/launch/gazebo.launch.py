import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = 'all_wheel'
    package_name = 'all_wheel_description'
    urdf_name = "all_wheel.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)
    gazebo_world_path = os.path.join(pkg_share, 'world', 'bot.world')

    # 关键：让 Gazebo 能找到你包里的 model / map
    # 你的 map 在 world/map 下（里面有 model.config 和 model.sdf），要把 world 目录加入 GAZEBO_MODEL_PATH
    gazebo_model_path = os.path.join(pkg_share, 'world')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # 可选但强烈建议：禁用在线模型库，避免卡在 models.gazebosim.org
    disable_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path
        ],
        output='screen'
    )

    # spawn 加 timeout，避免 30s 不够
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

    # 关键：等 Gazebo 进程启动后再延迟几秒 spawn（给它加载 world/model 的时间）
    delayed_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=start_gazebo_cmd,
            on_start=[TimerAction(period=5.0, actions=[spawn_entity_cmd])]
        )
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        output='screen',
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(set_gazebo_model_path)
    ld.add_action(disable_model_db)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(delayed_spawn)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld

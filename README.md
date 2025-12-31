# 全向底盘 Nav2 + SLAM

本工作区已包含 `all_wheel_navigation`，提供 SLAM（slam_toolbox）和 Nav2 的启动与配置。

## 源码编译 Nav2

以下步骤在本工作区内从源码编译 Nav2。请根据你的系统调整 ROS 发行版与分支。

```
# From /home/shexingju/code/ROS/ALL_WHEEL_CHASSIS
source /opt/ros/<distro>/setup.bash

mkdir -p src
vcs import src < https://raw.githubusercontent.com/ros-planning/navigation2/<branch>/nav2.repos

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

注意：
- 将 `<distro>` 和 `<branch>` 替换为你的 ROS 2 发行版（例如 `humble`）。
- 上述步骤需要网络访问以下载 Nav2 仓库。

## 运行 SLAM + Nav2（建图）

这会启动 Gazebo、机器人、slam_toolbox，以及 Nav2（SLAM 模式）：

```
ros2 launch all_wheel_navigation slam_nav2.launch.py
```

建图完成后保存地图：

```
ros2 run nav2_map_server map_saver_cli -f ~/maps/all_wheel
```

## 使用已保存地图运行 Nav2

使用保存的地图进行定位与导航：

```
ros2 launch all_wheel_navigation nav2_localization.launch.py map:=/path/to/map.yaml
```

## 小提示

- Nav2 配置期望雷达话题为 `scan`，里程计为 `omni_cmd_vel.py` 发布的 `/wheel_odom`。
- 若你已经在其它地方启动了机器人，请在导航启动文件中设置 `launch_sim:=false`。

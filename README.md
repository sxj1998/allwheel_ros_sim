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

## 建图导航（详细指令）

### 1) 终端环境（每个新终端都要执行）

```
source /opt/ros/<distro>/setup.bash
source install/setup.bash
```

说明：
- 第一行加载系统 ROS 2 环境。
- 第二行加载本工作区 overlay，确保 `all_wheel_*` 包可用。

### 2) 启动 SLAM + Nav2（建图模式）

```
ros2 launch all_wheel_navigation slam_nav2.launch.py
```

说明：
- 默认启动 Gazebo 仿真、机器人模型、slam_toolbox 与 Nav2（SLAM 模式）。
- 如需在真实机器人上建图（机器人本体与传感器已自行启动），请使用：
  `ros2 launch all_wheel_navigation slam_nav2.launch.py launch_sim:=false use_sim_time:=false`

### 3) 键盘遥控机器人建图

```
ros2 run all_wheel_description keyboard_teleop.py
```

说明：
- 该脚本发布 `/cmd_vel`，配合 `omni_cmd_vel.py` 输出 `/wheel_odom`。
- 常用按键：`w/s` 前后，`a/d` 旋转，`q/e/z/c` 斜向，`空格` 停止。

### 4) 保存地图

```
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/all_wheel
```

说明：
- 会生成 `~/maps/all_wheel.yaml` 与 `~/maps/all_wheel.pgm`。
- 若你在仿真中使用 `use_sim_time:=true`，可加参数：
  `ros2 run nav2_map_server map_saver_cli -f ~/maps/all_wheel --ros-args -p use_sim_time:=true`

### 5) 使用已保存地图运行 Nav2

```
ros2 launch all_wheel_navigation nav2_localization.launch.py map:=/path/to/map.yaml
```

说明：
- `map:=` 需指向上一步保存的 `*.yaml` 文件路径。
- 若你已经在外部启动机器人，请加：`launch_sim:=false`。
- 启动后在 RViz2 中用 **2D Pose Estimate** 设置初始位姿，再用 **Nav2 Goal** 发送导航目标点。

## Cartographer 建图（all_wheel_cartographer）

### 1) 编译（首次或更新后）

```
# From /home/shexingju/code/ROS/ALL_WHEEL_CHASSIS
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install --packages-select all_wheel_cartographer all_wheel_description
source install/setup.bash
```

### 2) 启动 Gazebo（终端 1）

```
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch all_wheel_description gazebo.launch.py
```

说明：
- 若 Gazebo 已在运行，请先关闭旧进程或使用不同端口。

### 3) 启动 Cartographer（终端 2）

```
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch all_wheel_cartographer cartographer.launch.py
```

说明：
- 默认使用 `/scan` 激光话题与 `/wheel_odom` 里程计话题。
- 追踪坐标系为 `base_link`，发布 `map -> odom`。

### 4) 键盘遥控建图（终端 3）

```
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run all_wheel_description keyboard_teleop.py
```

### 5) 保存地图

```
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/all_wheel
ros2 run nav2_map_server map_saver_cli -t /map -f hourse_map
```

说明：
- 会生成 `~/maps/all_wheel.yaml` 与 `~/maps/all_wheel.pgm`。
- 若你在仿真中使用 `use_sim_time:=true`，可加参数：
  `ros2 run nav2_map_server map_saver_cli -f ~/maps/all_wheel --ros-args -p use_sim_time:=true`

## 小提示

- Nav2 配置期望雷达话题为 `scan`，里程计为 `omni_cmd_vel.py` 发布的 `/wheel_odom`。
- 若你已经在其它地方启动了机器人，请在导航启动文件中设置 `launch_sim:=false`。




ros2 launch all_wheel_description gazebo.launch.py

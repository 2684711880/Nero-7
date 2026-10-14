# RANGER AIR 模型展示与移动控制

`model_preview.launch.py` 仅展示模型，没有运动控制器或里程计。Gazebo 默认暂停，避免无控制器的轮组自由运动。不要将这个预览作为底盘动力学验证。

移动控制已增加独立入口 `control.launch.py`，包括四轮独立转向、轮速控制、关节反馈、轮式里程计与 RViz。操作与验证数据见 [CONTROL.md](CONTROL.md)。

默认同时打开 RViz2。RViz 使用零位关节状态和墙钟时间，使 Gazebo 暂停时仍能显示完整 TF；这不是 Gazebo 关节反馈。后续接入控制器时须移除预览 joint_state_publisher，改用 joint_state_broadcaster，并统一使用仿真时间。只开 Gazebo 可传 `use_rviz:=false`。RViz 左侧勾选 TF 可查看坐标系。

模型及网格来自 AgileX 官方 ugv_gazebo_sim master，提交 `27633a956c845903ee630538afeb17fe70afdd84` 的 `ranger_air/ranger_air_description`；保持原 URDF 和 meshes。官方原包 package.xml 声明 BSD。本地新增 ament 构建文件、ROS 2 启动文件和展示场景。

来源：https://github.com/agilexrobotics/ugv_gazebo_sim/tree/27633a956c845903ee630538afeb17fe70afdd84/ranger_air/ranger_air_description

编译（当前终端使用 zsh）：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select ranger_air_description ranger_air_gazebo
source install/setup.zsh
```

Gazebo 展示（先停止其他 Gazebo 实例）：

```bash
ros2 launch ranger_air_gazebo model_preview.launch.py
```

结束时在启动命令所在终端按 Ctrl-C，使 gzserver、Gazebo 窗口和 RViz 一起退出。只关闭图形窗口可能留下后台 gzserver，导致下次启动提示 Gazebo 已在运行。使用 bash 的终端才加载 setup.bash；zsh 终端加载 setup.zsh。

独立 RViz 展示（提供零位关节状态，仅用于模型预览）：

```bash
ros2 launch ranger_air_description display.launch.py
```

两个预览入口请分别运行，避免同名 robot_state_publisher 及 robot_description 冲突。后续底盘控制应在 ranger_air_gazebo 中补齐，组合模型由单独整车描述包引用。

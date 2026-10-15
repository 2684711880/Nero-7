# 在 RViz 显示规划路径和末端反馈轨迹

整车RViz配置已启用 `Arm Trajectories` MarkerArray显示，话题 `/arm_trajectory_markers`：

- 绿色：MoveIt输出的关节轨迹通过正运动学计算出的末端规划路径。
- 橙色：执行时以约50 Hz读取关节反馈生成的末端TF，记录得到的路径。
- RViz Fixed Frame：`odom`；线条自身在`base_link`下记录，并随车体坐标系显示，
  与当前画圆代码的坐标含义一致，不把底盘的微小漂移计入机械臂圆轨迹。

橙色是反馈经模型计算的轨迹，不是Gazebo真值测量或真实机器人外部测量。
线/圆弧段之间保留轨迹，两个半圆叠加成一整圈；move_j/move_p准备动作会清除该臂上次轨迹。
执行结束后保持最后结果；最多保留约20000个反馈点，超过后丢弃最早一半。
新打开的RViz也可通过Transient Local QoS收到最后完整的两臂MarkerArray快照。

## 推荐：只打开RViz这个3D窗口

先Ctrl+C关闭旧仿真，加载重新编译的执行端和RViz配置：

```zsh
cd /home/a/Nero-7
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=65
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
ros2 launch mobile_dual_nero gazebo_moveit.launch.py \
  environment:=warehouse gui:=false use_rviz:=true
```

另一终端运行画圆C++程序：

```zsh
cd /home/a/Nero-7
export ROS_DOMAIN_ID=65
bash scripts/sim_commander.sh --ros-args -p send:=true
```

在RViz左侧勾选 `Arm Trajectories`。展开Namespaces可分别开关left_planned/left_feedback等线条。
如果希望看规划的机器人姿态动画，展开 `MotionPlanning → Planned Path`，
它订阅 `/display_planned_path`。这是规划回放，不是反馈轨迹；Show Trail显示的是机器人姿态残影。

## 已有仿真只补开RViz

确认运行的是新执行端后，在同一个ROS_DOMAIN_ID下执行：

```zsh
source /opt/ros/humble/setup.zsh
source /home/a/Nero-7/sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=65
ros2 launch mobile_dual_nero trajectory_view.launch.py
```

如果有自定义mount_xyz/mount_rpy，RViz启动也应传入相同安装参数。
旧进程不会因重新编译自动加载轨迹发布代码，必须重启执行端。

新增显示代码在 `sim_ws/src/mobile_dual_nero/src/official_command_bridge.cpp`，
不会改变commander.cpp里的画圆接口。验证记录见 `sim_ws/validation/trajectory_display/`。

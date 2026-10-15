# 用自己的 C++ 发布程序控制 Gazebo 机械臂

当前学习入口已简化为只画直线，先看 [最小直线程序](cpp_straight_line_first.md)。
本页保留完整参考版本的调用，使用`commander_reference`。运行本页命令前需在终端中
source `/opt/ros/humble/setup.zsh`、`sim_ws/install/setup.zsh`、`hardware_ws/install/setup.zsh`。

数据链：

```text
hardware_ws/.../commander.cpp
  → /left|right/control/move_l  (PoseStamped)
  → /left|right/control/move_c  (PoseArray，3个点)
  → mobile_dual_nero/official_command_bridge
  → MoveIt笛卡尔路径、碰撞检查、时间参数化
  → 左右臂JointTrajectoryController
  → Gazebo
```

发布接口沿用官方ROS消息格式，仿真接收端是本项目新增适配，未运行SDK/CAN。
它不模拟厂商控制器内部算法。仿真会依据frame_id做TF转换；官方实机驱动不会自动做这个转换。
因此把同一个任务转到实机前，必须适配单臂基坐标系和实际TCP。

## 构建与启动

仓库根目录，分别构建：

```zsh
cd /home/a/Nero-7
bash scripts/build.sh
bash scripts/build_official_control.sh
```

终端1启动仿真。当前已有窗口时无需重复启动：

```zsh
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=65
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
ros2 launch mobile_dual_nero gazebo_moveit.launch.py environment:=empty
```

新启动的仿真默认包含command_bridge；`command_bridge:=false`可关闭。
对旧版已启动仿真，可单独执行 `ros2 launch mobile_dual_nero official_command_bridge.launch.py`，
但同一个ROS domain中只运行一个接收端。它读取的安装参数需与整车一致。

## 运行 C++ 控制程序

终端2，在仓库根目录运行。每条命令等到SUCCEEDED后再执行下一条：

```zsh
cd /home/a/Nero-7
export ROS_DOMAIN_ID=65

# 仍然是自己的 C++ 发布程序；named_target是仿真扩展，先到准备姿态
ros2 run nero_official_commander_cpp commander_reference --ros-args -r __ns:=/left \
  -p motion:=named -p target:=ready -p send:=true -p wait_result:=true

# 从当前末端位置，在odom坐标系向-Z画1 cm直线，姿态保持不变
ros2 run nero_official_commander_cpp commander_reference --ros-args -r __ns:=/left \
  --params-file hardware_ws/src/nero_official_commander_cpp/config/sim_left_line.yaml \
  -p send:=true -p wait_result:=true

# 从当前点开始画半径5 mm的YZ半圆
ros2 run nero_official_commander_cpp commander_reference --ros-args -r __ns:=/left \
  --params-file hardware_ws/src/nero_official_commander_cpp/config/sim_left_arc.yaml \
  -p send:=true -p wait_result:=true
```

`--params-file`放在显式`-p send:=true`之前；配置文件默认send=false。
右臂使用`__ns:=/right`、`tcp_link:=right_gripper_base`，圆弧应把两个Y偏移改为正值。
若只改namespace却保留左臂tcp_link，目标仍会根据左臂当前TF生成，不是正确的右臂相对命令。

## 修改控制代码的位置

- 发布端：`hardware_ws/src/nero_official_commander_cpp/src/commander.cpp`。
  `moveLine()`、`moveArc()`负责发布，`main()`目前从参数读取动作。
  后续任务代码可以直接调用这些函数。
- 输入配置：同包`config/sim_left_line.yaml`和`sim_left_arc.yaml`。
  relative模式通过TF读取当前末端并加offsets；不是按上一次命令假设当前位置。
  绝对位姿仍用poses参数，每个点7个数：x,y,z,qx,qy,qz,qw。
- 仿真接收与执行：`sim_ws/src/mobile_dual_nero/src/official_command_bridge.cpp`。
  订阅回调只接收命令，工作线程执行规划，ROS线程持续处理MoveIt响应。

接收端串行处理左右臂动作，忙时拒绝新命令，不排队或抢占。
每个控制流只运行一个发布程序；当前String结果没有request_id，不能用于多客户端任务关联。
`control/result`提供ACCEPTED、EXECUTING、SUCCEEDED、FAILED、REJECTED状态。
wait_result只适用于本项目仿真接收端，官方实机驱动没有此结果topic。
超时表示结果未知，不会取消已经发布的运动；当前未实现任务取消。

直线/圆弧路径必须完整规划成功，否则不会执行部分轨迹。
圆弧要求当前末端已在起点附近（2 mm、0.01 rad），不隐式移动到起点。
关节采样跳变超过0.2 rad拒绝，关节速度/加速度缩放0.8，执行后检查终点关节误差0.02 rad。
当前TCP仍为gripper_base，不是画笔尖。底盘应保持停止，双臂同步和全身运动未实现。

## 复验

已启动同domain仿真后：

```zsh
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
source hardware_ws/install/setup.zsh
export ROS_DOMAIN_ID=65
python3 scripts/verify_cpp_topic_control.py
```

脚本实际运行C++发布程序，左右臂依次执行ready、直线、半圆，读取仿真末端TF验证位移；
最后发送不可达目标并检查拒绝执行。报告位于`sim_ws/validation/cpp_topic_control/`。

2026-10-14 已运行以上C++发布链：六项动作及一项不可达目标拒绝检查全部通过。
直线终点位移误差左/右臂约0.189/0.074 mm，半圆终点位移误差约0.215/0.029 mm。
这些是当前静止底盘、小范围路径的仿真TF采样结果，不代表沿途误差上界或实机精度。

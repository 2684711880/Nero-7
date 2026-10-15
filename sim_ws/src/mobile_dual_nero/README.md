# AIR + 双 NERO 整车仿真

组合包复用已有 AIR 与双 NERO 模型。默认将双臂支架安装在 AIR 坐标系 `(0, 0, 0.275)` m，安装姿态为零；这是仿真安装方案，尚未按实车安装尺寸校准。

使用一套 robot_description、controller_manager 与 joint_state_broadcaster。移除双臂原有 world 固定关节，保留左右机械臂和夹爪的原有关节名，将双臂原 base_link 重命名为 dual_nero_base_link；相机冲突名称也做映射。模型树为 `odom → base_link(AIR) → arm_mount_plate → dual_nero_base_link → 左右机械臂`。

## 运行（zsh）

整车运行时只需启动下面这一套，不要同时在同一 ROS 域内启动独立双臂或独立底盘。

终端 1：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=0
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
ros2 launch mobile_dual_nero gazebo_moveit.launch.py
```

终端 2 控制底盘：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 run ranger_air_gazebo air_keyboard --ros-args -p speed:=0.1 -p turn:=0.2
```

W/S 前后、A/D 左右横移、Q/E 原地旋转、空格停车。按运动键后持续运动，空格停止。Ctrl-C 退出键盘；终端 1 Ctrl-C 结束整套仿真。之前独立小车使用过域 37，整车默认使用域 0，因此键盘终端也必须切换到域 0。

底盘停车后，在 RViz 的 MotionPlanning 中选择 left_arm、right_arm、both_arms、left_gripper 或 right_gripper 规划组，选择 home/ready 或 open/close，然后 Plan / Execute。也可以拖动单臂末端交互标记进行位姿规划。当前验证流程为停车操作双臂；尚未实现底盘与双臂的同步全身规划或自动抓取。

安装参数可覆盖（单位 m/rad）：

```zsh
ros2 launch mobile_dual_nero gazebo_moveit.launch.py \
  mount_xyz:="0 0 0.275" mount_rpy:="0 0 0"
```

`mount_xyz` 表示双臂支架底部在 AIR base_link 中的位置，适配板厚 12 mm，位于安装面下方。调整安装位置后，应重新做碰撞、稳定性和规划验证。

## 实现与边界

- 保留原独立模型和启动入口；组合模型由 Python 构建，不复制一份容易分叉的大型 URDF。
- 整车 26 个控制关节：底盘 8、双臂 14、夹爪 4；一个 gazebo_ros2_control 硬件系统和一个 controller_manager。
- 七个控制器：joint_state_broadcaster、steering_controller、wheel_controller、左右臂和左右夹爪轨迹控制器。
- `/cmd_vel` 控制底盘；原有 FollowJointTrajectory 控制双臂和夹爪。
- `/joint_states` 来自实际 Gazebo 关节反馈。轮式里程计发布 odom → base_link，robot_state_publisher 发布安装链与机械臂 TF。所有节点使用仿真时间。
- MoveIt 通过 odom → base_link 的平面虚拟关节接收底盘位姿，底盘轮组标为 passive，不加入机械臂规划组。底盘驶离起点后仍可规划机械臂。
- MoveIt 地面固定在 odom 中，地面接触仅豁免轮胎；底盘、支架、机械臂和夹爪仍参与地面碰撞检查。原双臂自碰撞豁免表经过名称映射复用。
- RViz 固定坐标系为 odom，相机跟随 base_link，视距覆盖整车。
- Gazebo 启动会将 AIR、双臂及组合包的 share 目录加入客户端模型搜索路径。isolated colcon 安装中，这些目录分别位于不同包前缀；漏加会出现 RViz 正常而 Gazebo 只有适配板方块，并在客户端日志中报 model:// 网格找不到。
- 总模型质量约 32.35 kg（包含 0.6 kg 适配板）。这只是源模型质量之和，未验证 AIR 实车承载能力或惯量标定。
- 整车默认平移限速 0.2 m/s、角速度限速 0.4 rad/s；加速度分别为 0.15 m/s²、0.3 rad/s²；转向限速 0.8 rad/s；保留指令和关节反馈超时停止。
- 底盘采用简化接触碰撞，忽略部分传感器与转向支架碰撞；驱动与机械臂为 GazeboSystem 的理想速度/位置接口。尚未验证大幅伸臂时的抗倾覆边界、载物、坡道、传感器输出或真实硬件。
- 当前 MoveIt 版本在 SIGINT 关闭时可出现析构段错误；运行期间的规划执行已验证，关闭阶段异常未在本包内修复。

## 验证

记录目录：`/home/a/Nero-7/sim_ws/validation/mobile_dual_nero/`。

- `run_01/assembly.json`：Gazebo 中底盘和两侧腕部一起平移约 0.467 m，刚性安装关系通过。
- `run_01/arms.json`：装车后左右臂、双臂 ready/home、两侧夹爪开合及两侧末端 IK 规划，共 12 项检查通过。
- `run_01/motion/`：保留首次发现固定时间步长问题的失败记录。
- `run_02/motion/report.json`：修正控制定时后，带双臂底盘 8 项运动与停车测试通过，含独立 Gazebo 真值和轮式里程计比较；`telemetry.csv` 保存完整测量。
- `run_02/arms.json`：整车平移、旋转并停下后再次执行机械臂验证的 12 项通过记录。运行控制台日志不纳入版本管理。
- `colcon test` 覆盖模型树、唯一关节/控制系统、SRDF 引用、安装参数、底盘运动学和控制定时降频回归。

重新验证前先停车并关闭键盘，确认没有其他 `/cmd_vel` 发布者。验证命令在整车同一 ROS 域中执行：

```zsh
ros2 run mobile_dual_nero verify_assembly \
  --output /home/a/Nero-7/sim_ws/validation/mobile_dual_nero/new_run/assembly.json \
  --ros-args -p use_sim_time:=true

ros2 run ranger_air_gazebo verify_motion --entity mobile_dual_nero \
  --output-dir /home/a/Nero-7/sim_ws/validation/mobile_dual_nero/new_run/motion \
  --ros-args -p use_sim_time:=true

ros2 launch mobile_dual_nero verify_arms.launch.py
```

如果修改了安装参数，`verify_arms.launch.py` 也须传入相同 mount_xyz/mount_rpy。

## 货物整理环境

默认启动 `warehouse` 环境，含两组四层货架、两张工作台、一瓶动态水瓶和六个静态示例货箱。使用 `environment:=empty` 恢复原平地环境。尺寸及位置见 `mobile_dual_nero/environment.py`，Gazebo 和 MoveIt 使用同一几何定义。水瓶位姿从 `/gazebo/model_states` 同步到 MoveIt；初始 world 与 odom 对齐，底盘轮式里程计漂移尚未校正。抓取、搬运及导航尚未验证。

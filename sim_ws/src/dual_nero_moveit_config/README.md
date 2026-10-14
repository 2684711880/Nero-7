# 双 NERO：Gazebo Classic + MoveIt2（ROS 2 Humble）

本地适配包，复用 `smalleha/isaac_so_arm101` 的现成双臂模型；不是上游官方 Gazebo 双臂发行包。
模型来源和修改见 ASSET_SOURCE.md。上游侧装布局，左右肩安装点为 (-0.002, ±0.1, 0.59) m。

## 编译与启动（zsh）

先停止已有单臂仿真，避免 Gazebo master、robot_description、controller_manager 重名。

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select dual_nero_moveit_config
source install/setup.zsh
source /usr/share/gazebo/setup.sh
LC_NUMERIC=C ros2 launch dual_nero_moveit_config gazebo_moveit.launch.py
```

RViz 的 Planning Group 可选 `left_arm`、`right_arm`、`both_arms`、`left_gripper`、`right_gripper`。
先选择 `left_arm`，Goal State 选择 `ready`，点击 Plan，再 Execute；再试右臂和 both_arms。
`home` 为各臂零关节角，`ready` 为两臂 joint4=0.3 rad；夹爪有 open / close。
`both_arms` 的关节/命名目标规划由统一规划场景检查两臂碰撞；末端交互标记先选单臂组。

## 验收

```zsh
ros2 control list_controllers
ros2 topic echo /joint_states --once
# 自动检查左右臂和双臂规划、执行以及反馈误差。只在本仿真中运行。
ros2 run dual_nero_moveit_config verify_dual --ros-args -p use_sim_time:=true
```

预期五个 active 控制器：joint_state_broadcaster、left_arm_controller、right_arm_controller、
left_gripper_controller、right_gripper_controller。18 个运动关节：14 个臂关节 + 4 个夹爪关节。
相机只有几何模型，没有图像输出；顶部相机支架固定在零位。固定底座与 world 连接。

同臂碰撞豁免继承当前单臂 SRDF，并补固定/相邻装配件豁免；移动左右臂之间的碰撞检测保留。
此模型未做实机标定，惯性/接触参数未做实测验证。当前用途是规划与仿真执行。

## 抖动恢复与地面碰撞（2026-10-14 修复）

此前 MoveIt 场景缺少地面，允许把夹爪规划到 Gazebo 地面以下；位置控制和地面接触相互冲突，造成持续振荡。
当前启动会先将 z=0 的地面加入 MoveIt，成功后才打开 RViz。仅固定底座 base_link 与地面接触被允许，机械臂/夹爪与地面仍检查碰撞。
启动前还会检查 Gazebo master，已有实例时拒绝重复启动。物理步长、重力、ODE 求解器与执行起点容差保持原配置。

若已经进入抖动状态：在启动仿真的终端 Ctrl-C，等进程全部退出，再按上面的命令启动；只点 Reset World 不足以清除控制器保持的旧目标。
RViz 的 Start State 使用 Current State；先选 ready 验证，再拖动末端。红色/碰撞目标不要执行。

```zsh
# 只读：测量静止状态 6 秒内的抖动幅度
python3 src/dual_nero_moveit_config/scripts/check_jitter.py
# 反例：确认历史穿地姿态无法通过规划（不执行运动）
python3 src/dual_nero_moveit_config/scripts/check_ground.py
```

通过直接向控制器发轨迹可绕过 MoveIt 的碰撞检查；本修复保证的是通过 MoveIt 规划的路径。

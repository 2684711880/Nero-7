# C++ 双桌搬瓶测试（Gazebo 专用）

主程序：`hardware_ws/src/nero_official_commander_cpp/src/pick_place.cpp`。
保留原来的 `commander.cpp` 画圆示例。

## 运行

重新启动仿真；已有仿真应在其终端 Ctrl+C 停止，避免两个底盘控制器同时运行。

终端 1（zsh）：

```zsh
cd /home/a/Nero-7
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=65
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
ros2 launch mobile_dual_nero gazebo_moveit.launch.py environment:=warehouse gui:=true use_rviz:=true sim_grasp:=true ground_truth_odometry:=true
```

等待 `READY: official-schema topics`，终端 2：

```zsh
export ROS_DOMAIN_ID=65
bash /home/a/Nero-7/scripts/sim_pick_place.sh -p send:=true
```

不传 `send:=true` 仅打印流程，不发送运动。再次测试请重启世界，恢复瓶子初始位置。
没有连接真实 CAN/硬件驱动，不要把这个程序用于实机。

## 任务顺序

1. 双臂收臂，右爪张开，右臂升到车体附近的高位准备姿态。
2. 底盘通过 `/cmd_vel` 闭环停靠源桌，目标 `(0, -0.80, 0)`。
3. 夹爪水平朝向瓶身，右臂移到夹持点上方，直线下降，夹爪开口收至 70 mm。
4. 检查夹取位置和开口，Gazebo 建立固定连接；MoveIt 将瓶子附加到右爪。
5. 直线上提，移到运输姿态，检查瓶子随爪移动。
6. 退到中央，转向 180°，停靠另一张桌子 `(0, +0.80, π)`。
7. 移到放置点上方，直线下降，解除固定连接，松爪并向上撤离。
8. 检查瓶子在目标位置、落在桌面且保持竖直。

所有坐标以 `odom` 为参考，长度单位 m。瓶子中心来自 `/gazebo/model_states`。
夹取点按 `right_gripper_base` 的局部 +Z 方向偏移 0.18 m 定义；此值是本模型的夹持中心，不是通用工具参数。
当前工作站和瓶子尺寸固定：初始瓶子 `(0, -1.60)`，目标 `(0, +1.60)`，瓶高 0.24 m、直径 0.068 m。
桌面高度取自初始瓶子中心减去半瓶高，保持与 `environment.py` 的桌高一致。

## 接口和仿真边界

- 机械臂复用 `ArmCommander::move_j/move_p/move_l`，通过官方样式 ROS 消息到现有 MoveIt 仿真桥。
- 新增 `ArmCommander::gripper(width, force)`，通过官方 `move_j` 的仅 `gripper` 关节消息控制；宽度为总开口。仿真使用位置控制，不模拟指定夹紧力。
- 底盘驱动仍是现有 AIR 控制器。`drive()` 封装闭环停靠，后续可以替换其传输实现。
- `ground_truth_odometry:=true` 用 Gazebo 物理位姿代替轮式积分作为定位，**不修改物理位置**；默认关闭，其他仿真保持原来行为。
- `sim_grasp:=true` 加载 `bottle_grasp_plugin.cpp`。夹持中心距瓶心不超过 25 mm、夹爪总开口在 60–75 mm 时才允许建立固定连接。没有物体传送、没有持续改写瓶子位姿。
- 固定连接是简化抓取模型，不验证摩擦、夹紧力或滑落。松开后由 Gazebo 重力和桌面碰撞决定落稳结果。
- MoveIt 只允许瓶子与右夹爪接触；搬运时瓶子作为附着物体参与规划。
- 路线是已知通道内的固定停靠序列，没有接入导航避障。底盘转移与机械臂运动依次执行；停靠后底盘持续闭环保持位置，抵消物理漂移。
- 任务出错即停止后续步骤，发布底盘零速度，不自动重试或释放已夹物体。机械臂命令超时不等同于取消执行。

## 验证

验收日志和真值采样放在 `sim_ws/validation/pick_place/`。
`report.json` 仅在整条流程通过后生成；`trace.json` 由独立只读观察脚本记录。

2026-10-15 验收结果：

- 完整取放执行通过；瓶子从 `(0, -1.60, 0.72)` 搬到约 `(0.000186, 1.598886, 0.720000)`。
- 最终水平误差 1.13 mm，高度误差约 0.01 mm，瓶子保持竖直；运输高度约 0.949 m。
- 独立 Gazebo 真值记录包含 2505 个完整样本；338 个行驶样本中的瓶子相对车体位置变化小于 0.10 mm。
- 远离瓶子时的固定连接请求被拒绝；未恢复初始底盘位置时的任务请求被拒绝。
- 场景同步专项测试通过：瓶子附着期间没有世界物体 MOVE 消息，解除附着后恢复更新。这个专项只修改规划场景，不执行物理运动。
- 回归检查：16 tests，0 errors，0 failures。

完整搬运验收在位置同步降噪修复前执行；该修复随后通过单元测试和运行中的规划场景专项验证。

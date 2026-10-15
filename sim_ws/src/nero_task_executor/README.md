# 三路机器人动作任务（第一版）

上层发布 JSON；底盘、左臂、右臂分别顺序执行，三路并行。`wait_for` 只增加跨动作依赖；序列本身隐含前后依赖。任务执行器不生成额外回零、准备或撤离动作。

## 文件入口

- `examples/bottle_transfer.json`：水瓶任务，初始三路并行，`r3` 等底盘到位，`b2` 等抬起，`r7` 等第二次到位，`b3` 等松开后返回起点。
- `nero_task_executor/model.py`：完整输入校验、循环依赖检查。
- `nero_task_executor/engine.py`：不依赖 ROS 的三路状态机。
- `nero_task_executor/node.py`：ROS 接收、反馈、取消、查询、持久化。
- `nero_task_executor/sim_backend.py`：底盘闭环、仿真夹持与规划场景同步。
- `src/sim_arm_server.cpp`：每臂独立 ROS Action，MoveIt 规划，独立控制器执行。

这是 Python 调度节点 + C++ 仿真运动适配器。保留原有 ArmCommander/官方话题接口；新适配器复用其消息语义和 MoveIt 运动能力。旧接收端全局 busy，且共享 ExecuteTrajectory 不适合此处独立双臂执行，因此新增带 goal ID 的执行通道。

## 构建与无运动验证

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select nero_task_interfaces nero_task_executor
source install/setup.zsh
ros2 run nero_task_executor task_executor
```

另一终端：

```zsh
source /home/a/Nero-7/sim_ws/install/setup.zsh
ros2 run nero_task_executor submit_task \
  /home/a/Nero-7/sim_ws/src/nero_task_executor/examples/bottle_transfer.json
```

默认 `backend=mock`，只验证动作流，不发布机器人运动指令。反馈含 `execution_mode=mock`，成功不能作为抓取或实机证明。相同 task_id 和相同内容只返回已有状态；有意重新执行使用 `--task-id bottle_transfer_002`。

## 仿真执行

在没有其他 Gazebo 运行的情况下，第一终端：

```zsh
source /home/a/Nero-7/sim_ws/install/setup.zsh
ros2 launch mobile_dual_nero gazebo_moveit.launch.py \
  command_bridge:=false sim_grasp:=true ground_truth_odometry:=true
```

第二终端，模型与控制器就绪后：

```zsh
source /home/a/Nero-7/sim_ws/install/setup.zsh
ros2 launch nero_task_executor simulation.launch.py
```

第三终端先发送 `examples/parallel_smoke.json`，确认非零底盘位移和双臂准备运动。仿真服务器只接受 `use_sim_time=true`；本包没有实机 backend。

水瓶任务已在重置场景完成一次完整仿真验收，水瓶最终水平误差约 24 mm，竖直程度 0.999999999；结果见 `sim_ws/validation/task_executor/bottle_report.json`。这是当前场景与初始状态的一次验证，不是任意起始姿态或实机保证。

水瓶 JSON 的数值根据现有桌高 0.6 m、水瓶中心 (0,-1.6,0.72)、源停靠 (0,-0.8,0)、目的停靠 (0,0.8,pi) 的场景整理，已完成上述场景的仿真验证；仅通过 mock 的其他参数不能认定抓放成功。修改 TABLE_HEIGHT 或停靠位置后必须同步更新目标。左臂仅给 J2 设置 +60°（pi/3 rad）目标，J1 和其余关节保持动作开始时的位置；单关节修改不会自动实现整臂折叠。`r1` 表达准备 TCP 位姿而非指定 J4；IK 不保证只转动 J4。

`b2.parameters.via` 明确给出中心平移、原地转向的途经点；底盘仅沿给定点闭环行驶，没有障碍规划。双臂分别规划，不提供联合动态避碰；底盘移动时的机械臂规划也只是当时场景的快照。先在仿真确认这组并行路径。

新增独立组合运动任务 `examples/bottle_transfer_combined.json`，注释见同名 `.jsonc`。原任务保留。新任务 b1 使用 `base_translate` 保持初始朝向靠近源桌；b2 等右臂抬起后，使用 `base_move_and_rotate` 同时移动到 `(0,0.8)` 并转到 π；b3 等松瓶后，同时返回 `(0,0)` 并转回 0。新路线没有中央途经停靠点。机械臂目标、左臂 J2 +60° 和全部跨序列依赖与原任务一致。

从重置的默认场景出发，前两个终端启动完成后，第三个终端运行：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=75
ros2 run nero_task_executor submit_task \
  /home/a/Nero-7/sim_ws/src/nero_task_executor/examples/bottle_transfer_combined.json
```

三个终端的 ROS_DOMAIN_ID 须一致。原流程运行后水瓶已经在左桌，运行新流程前需重置场景，使水瓶恢复到源桌。

组合版本已完成一次完整 Gazebo 验收：搬运与回程的实际平移/旋转分别重叠约 30.1 s 和 20.1 s；放瓶水平误差约 5.5 mm，底盘回原点位置误差约 0.71 mm。记录见 `sim_ws/validation/bottle_combined/README.md`。

组合运动的后续平顺性修复：AIR 转向对齐改为按目标轮速加权、连续减速；C++ 停靠补偿保持合成速度方向，并在巡航时退出。相同任务的无界面对照中，搬运与回程中段的停轮占比从约 15.5% / 36.1% 降为 0% / 0%，完整搬运和取消验证通过。数据与速度曲线见 [平顺性诊断记录](../../../sim_ws/validation/base_smoothness/README.md)。

## 通信与状态

- `/robot_task/data` (`std_msgs/String`)：完整 JSON，最大 1 MiB、1000 个动作。
- `/robot_task/status` (`std_msgs/String`)：任务与逐动作状态，可靠、保留最近状态。
- `/robot_task/query` (`std_msgs/String`)：data 为 task_id。
- `/robot_task/cancel` (`std_msgs/String`)：data 为 task_id，取消整个任务。
- `/task_motion/left_arm`、`/task_motion/right_arm`：内部 ExecuteMotion Action，独立 goal ID 与取消结果。

```zsh
ros2 topic pub --once /robot_task/query std_msgs/msg/String '{data: bottle_transfer_004}'
ros2 topic pub --once /robot_task/cancel std_msgs/msg/String '{data: bottle_transfer_004}'
```

任务：RUNNING → SUCCEEDED，或 STOPPING → FAILED/CANCELED/STOP_UNCONFIRMED。动作：PENDING、RUNNING、SUCCEEDED、FAILED、BLOCKED、CANCELED、STOP_UNCONFIRMED。

任一路失败/超时停止整个任务，禁止后续下发，取消其他在途动作。取消 Action 接受不等于停止；等待控制器终态。底盘发送零速度后还要确认低速持续 0.4 s。停止结果未知会保留机器人占用、拒绝新任务，不自动重试或松开物体。

状态原子写入 `~/.local/state/nero_task_executor/{mock|simulation}.json`，也可通过 `state_file` 参数指定。重启恢复历史和去重；中断任务恢复为 STOP_UNCONFIRMED 并锁定。确认机器人已停止、抓持和场景状态一致后，人工归档旧状态文件，再启动新会话。不要用换状态文件绕过未确认的运动。文件锁防止同一状态文件被两个执行节点同时使用；同一个 ROS 域仍应只运行一个任务执行器，不要用其他控制节点同时驱动相同资源。

## 参数语义

- `move_j`：7 个 rad，J1..J7。
- `move_j_partial`：例如 `{"joint_positions_rad":{"J1":0.2}}`，动作开始时读取反馈补齐其他关节。
- `move_p`/`move_l`：m + 单位四元数 xyzw，frame_id 经 TF 转换，TCP 固定为 default_tcp（模型 gripper_base）。
- `move_c`：parameters.poses 为三个完整位姿；开始前必须已在起点。
- `gripper_open`：opening_m 为总开口 0..0.1 m。
- `gripper_grasp`：当前只实现右臂 water_bottle 仿真配置，开口 0.070 m，再请求已有 Gazebo 固定约束插件检查几何关系并附着，之后同步 MoveIt 附着物。不是摩擦夹持或实机力反馈证明。
- `touch_object: "water_bottle"`：显式允许水瓶与右夹爪三条 link 接触，不能取消桌面或其他物体碰撞检查。
- `base_move_to`：odom 坐标系目标 xy、yaw，可选 via=[[x,y,yaw],...]，需位置和停止速度同时达标。到位采用 12 mm / 0.015 rad 进入、25 mm / 0.03 rad 退出的滞回区间，再确认停止速度持续 0.4 s，避免边界噪声使计时反复重置。
- `timeout_s`：单个动作实际开始后的墙钟超时，等待依赖不计入；底盘 via 共享此超时。

## 测试

```zsh
cd /home/a/Nero-7/sim_ws
source install/setup.zsh
colcon test --packages-select nero_task_executor --event-handlers console_direct+
colcon test-result --verbose
```

已完成：24 项单元/回归测试，8 项真实 ROS 通信与重启检查，非零底盘位移及双臂执行 smoke，完整水瓶抓放仿真。完整抓放没有执行旧程序的右臂回零、额外上方接近、搬运姿态和放置后撤离动作。右臂命令从旧程序的 12 次减少为此任务显式的 7 次；不能据此推断未测量的耗时收益。

右夹爪与 water_bottle 的接触许可在当前仿真会话保留，其余碰撞对不因此放开。仿真结束时需停止 launch；执行器不会自动松开仍在抓持的物体。

已知运行限制：本次 MoveIt move_group 在停止整个仿真时出现退出阶段段错误（exit -11），完整任务和水瓶位置验收在此前已完成。详见验证记录；本包没有修复上游退出问题。

放置动作修正：已移除 r6 的 move_p 上方位姿重规划；r7 直接依赖 b2，底盘到位后直线下降，再执行 r8 松开。任务编号更新为 bottle_transfer_002，避免与旧内容使用相同编号冲突。旧版八动作流程的记录保留。直接下降版本已在独立仿真完成抓放，水平误差约 6.8 mm，下降阶段最大关节角跨度 0.335 rad，见 `sim_ws/validation/task_executor/direct_place/report.json`。

返回起点版本：task_id=bottle_transfer_003。b3 在 r8 松开完成后开始，先保持 pi 朝向平移到中心，再转回 yaw=0，最终 odom 的 xy=[0,0]。这对应当前场景的初始位置与朝向，不会自动采集任意起始位姿。左臂 J1=+60°。其余动作保留直接下降版本。

返回起点版本独立仿真已通过：底盘位置误差约 10.6 mm，朝向误差约 0.56°，左臂 J1 实测 59.99°。记录见 `sim_ws/validation/task_executor/return_home/report.json`。

当前版本为 bottle_transfer_004：左臂动作只包含 J2=+pi/3，J1 不再给目标。针对实际停在底盘转向末段的问题，平移收尾最低指令设为 0.04 m/s，转向最低指令设为 0.12 rad/s；最高值仍为 0.10 m/s、0.18 rad/s，进入到位区后发送零速度并等待停止反馈。保持原有到位滞回和实际反馈验证，不通过放宽容差或直接报成功绕过停滞。旧进程不会自动加载 Python 修改，需要在重置场景后重启执行节点。

J2 与停滞修正已验证：24 项测试、完整抓放返回，以及额外两次连续原地转向通过。水瓶放置误差约 2.9 mm，底盘返回误差约 5.3 mm，J2 实测 60.002°，J1 相对初始变化约 0.0055°。见 `sim_ws/validation/task_executor/j2_turn_fix/README.md`。

## 可选 C++ 底盘动作

保留原 `base_move_to`；新增 `base_translate`、`base_rotate`、`base_move_and_rotate` 共用 C++ 参考轨迹与位姿反馈核心。参数与运行方式见 [底盘动作说明](../../../docs/base_motion_modes.md)，示例为 `examples/base_modes.json`，可读注释版为 `examples/base_modes.jsonc`。水瓶任务未改为组合运动。

# 三路动作任务入口

实现与运行说明见 [nero_task_executor README](../sim_ws/src/nero_task_executor/README.md)。

上层发送 [水瓶任务 JSON](../sim_ws/src/nero_task_executor/examples/bottle_transfer.json)，执行节点独立推进底盘、左臂、右臂三条顺序序列，按 wait_for 协调。ROS 入口为 /robot_task/data，结果为 /robot_task/status。

默认 mock，不发送运动。仿真使用新增的带 goal ID 的左右臂 Action 通道；原有官方话题接口与旧抓放程序保留。调度核心是 Python，仿真机械臂适配器是 C++，实机适配器尚未接入。

完整水瓶抓放已在当前 Gazebo 场景验证，水平误差约 24 mm；[验收记录](../sim_ws/validation/task_executor/README.md)区分调度测试、仿真运动和未验证的实机能力。

放置已改为底盘到位后直接 move_l 下降再松开，移除上方位姿 move_p；新版 task_id=bottle_transfer_002。独立仿真水平误差约 6.8 mm，下降阶段最大关节角跨度 0.335 rad，见 [修正验收](../sim_ws/validation/task_executor/direct_place/README.md)。

新增 b3：等 r8 松开后返回 odom 原点并恢复 yaw=0；左臂 J1 目标 +60°。当前任务编号 bottle_transfer_003。

当前版本 bottle_transfer_004：左臂改为仅 J2=+60°，J1 保持；修复底盘低速收尾停滞。24 项测试、完整抓放返回及两次连续转向验证通过，见 [故障与验收记录](../sim_ws/validation/task_executor/j2_turn_fix/README.md)。旧进程须在重置场景后重启以加载修正。

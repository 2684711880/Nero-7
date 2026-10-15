# 三路任务验证记录

- `ros_report.json`：8 项 DDS 收发、去重、拒绝、取消及重启恢复检查，运动 backend 为 mock。
- `ros_events.json`：上述测试的逐任务与逐动作反馈。
- `simulation_smoke.log`：底盘 0.05 m 目标位移、左右臂 J4=1 的独立执行、依赖后右夹爪打开，SUCCEEDED。
- `bottle_report.json`：最终代码从重置场景直接执行水瓶任务；任务 SUCCEEDED，实际水瓶位置约 [0.00017, 1.57599, 0.72000] m，水平误差 0.02401 m，竖直程度 0.999999999。使用检查几何关系的 Gazebo 固定关节抓取，不证明实机摩擦夹持。
- `simulation_bottle.log`：完整任务的动作状态序列。
- `bottle_attempt1_canceled.json`：第一次测试暴露底盘容差边界计时反复重置，抓取抬起后取消；停止得到确认。修复滞回区间后重新从重置场景验证，旧证据保留。
- `simulation_state.json`：最终已完成任务的持久化状态。

运行期间使用独立 ROS_DOMAIN_ID=74（mock）和 75（Gazebo），没有连接 CAN 或启动官方实机驱动。验证结束已停止本次启动的仿真和执行节点。

单元/回归测试结果：19 passed，0 failed，覆盖三路并行、依赖、非法输入、失败阻断、超时、取消、停止未知锁定、旧回调隔离和底盘到位滞回。

退出阶段另外观察到上游 MoveIt move_group 在析构 CallbackGroup 时 exit -11；发生于完整抓放验收之后。本次未修改上游 MoveIt，也不把单次运动成功扩大为整个系统退出稳定性证明。执行节点自身的 ROS 上下文关闭后发布异常已修复，并独立复验退出。

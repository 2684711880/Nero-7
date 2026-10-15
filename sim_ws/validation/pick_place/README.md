# 搬瓶验收

- `report.json`：C++ 完整取放的成功判据与最终误差。
- `task.log`：完整取放八个阶段、两次夹持位置检查与最终 PASS。
- `trace.json`：独立 Gazebo 真值观测，无控制命令。
- `trace_acceptance.json`：真值记录统计。
- `rejected_distant_grasp.log`：夹爪远离瓶子时拒绝建立固定连接。
- `rejected_wrong_start.log`：未复位初始底盘位置时，任务在运动前拒绝执行。
- `dry_run.log`：默认不发送控制。
- `final_scene.log`：完整任务释放后的规划场景，无附着物体。
- `sync_runtime.json`：后续同步降噪修复的专项运行验证，只有规划场景附着/解除，没有物理运动。
- `regression.log`：16 tests，0 errors，0 failures。

抓取使用位置与开口校验后的 Gazebo 固定连接；不代表真实摩擦夹持验证。整条测试运行于隔离 ROS domain 72，未连接实机 CAN。

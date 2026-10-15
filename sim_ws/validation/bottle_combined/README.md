# 组合底盘夹水任务验收（2026-10-15）

任务：`bottle_transfer_combined_001`，输入为 `src/nero_task_executor/examples/bottle_transfer_combined.json`。

在独立 ROS_DOMAIN_ID=79、Gazebo Master 11347 的重置默认场景执行。未连接真实硬件。使用 checked_fixed_joint 仿真夹持；实际位置与速度来自 Gazebo ModelStates，不能视为实机抓取证明。

[完整观测报告](report.json) 的 `passed` 为 true，所有动作 SUCCEEDED。

- 抬瓶后到左桌的 b2：实际平移速度 >0.01 m/s 且角速度 >0.03 rad/s 累计约 30.07 s。
- 放瓶后回原点的 b3：同样的实际运动重叠约 20.11 s。
- 水瓶放置水平误差约 5.5 mm，竖直余弦约 1。
- 底盘返回原点位置误差约 0.71 mm，朝向误差约 0.0092 rad。
- 左臂 J1 改变量约 0.000032 rad，J2 的 +60° 目标误差约 0.000025 rad。
- 放置下降过程中，右臂各关节最大运动跨度约 0.379 rad，未出现整圈旋转。

编译、45 项单元测试、JSON 参数校验及 JSON/JSONC 一致性检查通过。原 `bottle_transfer.json` 未修改。

只验证了本次默认场景和初始状态；新任务不包含障碍规划。复现时先重置场景及水瓶位置，按 package README 启动场景和适配器，然后运行：

```zsh
python3 src/nero_task_executor/tests/observe_sim.py \
  --output /tmp/bottle_combined_report.json \
  --task-id bottle_transfer_combined_001 --return-home --expect-combined
```

在另一个加载相同 ROS 环境且 ROS_DOMAIN_ID 一致的终端提交该 JSON。

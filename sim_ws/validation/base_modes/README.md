# 底盘三种动作验证（2026-10-15）

新增 `base_translate`、`base_rotate`、`base_move_and_rotate`，保留旧 `base_move_to` 和原水瓶任务。

## 结果与证据范围

- C++ 控制核心及 Python 调度/参数检查：45 项测试通过。
- [协议报告](protocol/report.json)：真实 DDS、C++ Action 服务器及 Python 调度器，配合合成里程计；六项检查通过，覆盖三模式、旧新控制权交接、取消、超时、里程计失效和未知停止后的锁定。
- [Gazebo 三模式报告](gazebo/report.json)：读取 ModelStates 实际运动，平移与转向同时发生约 9.9 s；纯平移朝向偏移约 2e-8 rad，纯转向位置偏移约 0.03 mm。回原点误差约 0.36 mm / 0.0266 rad，符合最终制动容差 25 mm / 0.03 rad。
- [Gazebo 取消报告](gazebo/cancel_report.json)：运动中取消，任务变为 CANCELED，后续动作 BLOCKED；制动位移约 5.4 mm，最终实际速度约 0.00025 m/s / 0.00085 rad/s。
- [原水瓶流程回归](legacy_bottle/report.json)：同一 Gazebo 会话运行原任务成功，放瓶位置误差约 9 mm；底盘回原点约 1.6 mm / 0.005 rad。水瓶任务 JSON 未修改。

本次没有连接真实硬件，没有验证障碍规划或机械臂与环境的动态避碰。参考曲线的速度、加速度约束与实际物理轨迹应分别理解。

## 重跑

在 sim_ws 中完成 ROS 环境和 install/setup.zsh 加载后：

```zsh
colcon test --packages-select nero_task_executor
colcon test-result --test-result-base build/nero_task_executor --verbose
ROS_DOMAIN_ID=78 python3 src/nero_task_executor/tests/verify_base_protocol.py --output validation/base_modes/protocol
```

Gazebo 场景及适配器按项目原步骤启动，提交 `src/nero_task_executor/examples/base_modes.json`。观测脚本 `observe_base_modes.py` 和 `verify_base_cancel_gazebo.py` 位于该包 tests 中；运行前用 `--help` 查看输出路径参数。取消测试会主动提交及取消运动，应在独立、空闲的仿真会话执行。

# 抖动问题修复记录（2026-10-14）

故障目标见 jitter_repro.json。左夹爪关节坐标约 z=-0.11m，进入 Gazebo 地面。
旧 MoveIt 场景缺少地面，规划能通过，Gazebo 位置命令与接触约束冲突。
清除重复实例后，在隔离仿真中仍复现：6秒内关节最大摆幅 3.3459 rad（jitter_baseline.log）。

修复：add_ground.cpp 在 MoveIt 场景加入顶面 z=0 的地面；仅允许固定 base_link 与地面接触；场景成功初始化后才启动 RViz。
启动时检查已有 Gazebo master，阻止重复实例。
世界物理参数已恢复，与原单臂 world 文件字节一致；未放宽执行容差、关闭重力或取消地面接触。

验证：
- 历史穿地目标判定与地面碰撞，规划被拒绝（check_ground.log）。
- 10次正常规划执行 + 2次末端位姿规划通过（recovery_verify.log）。
- 6秒600次关节状态采样，静止关节摆幅小于 1e-6 rad（recovery_idle.log）。
- 重复启动被明确拒绝。

此前 VALIDATION.md 只覆盖零位附近，未检查地面目标，本记录补充该证据边界。
通过控制器 Action 直接下发轨迹仍可绕过 MoveIt 碰撞检查；上述保护适用于 MoveIt 规划链路。

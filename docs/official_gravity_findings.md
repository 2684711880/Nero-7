# NERO 官方重力补偿核对

日期：2026-10-23。仅源码、联网文档与离线动力学/SDK 指令验证；未操作实机。

## 来源

- 官方仓库：https://github.com/agilexrobotics/agilex-arm-gravity-compensation
- 本地固定版本：8e2545c00ec9ec7590414946ab5f913274c7b643
- 入口：`nero/main_gc.py`；计算：`core/agx_pinocchio.py`。

## 补偿实现

示例 200 Hz 读取 q、qd，以 qdd=0 调用 Pinocchio RNEA，产生 C(q,qd)qd+g(q)，经七轴 MIT 力矩前馈下发。kp=kd=0，因此 p_des=0 本身不产生回零的位置误差力矩。qd=0 时为纯重力项；运动时还包含速度相关项。

示例使用 NeroFW.V121，无 leader 调用。提交记录明确取消了启动初始位姿运动，但使能过程仍包含 SDK 的控制模式切换。

## 已验证的启动指令顺序

`log/gravity_environment/startup_trace.json` 来自真实 SDK 的离线调用与编码，模拟反馈，并禁止创建 socket，未发送到 CAN。

在初始以太网控制状态下：

1. 0x151：01 01 32 00 00 00 01 00；CAN 控制、MOVE_J、50% 速度字段。
2. 0x471：08 02 00 00 00 00 00 00；全轴使能。
3. 0x151：01 06 32 AD 00 00 01 00；切到 MIT。
4. 0x15A 等：关节 MIT 力矩命令。

50% 是模式帧中的速度设置字段，不代表已经测到机械臂以 50% 速度运动。该证据证明使能并非无控制状态变化的操作，不证明上次两次摆动的根因。

## 当前环境与模型

独立环境 `.venv-gravity`：Pinocchio 4.1.0、pyAgxArm、NumPy、SciPy。清除外部 PYTHONPATH 后 pip check 通过。

原版裸臂和配置后的标准夹爪模型均已离线加载并校验 RNEA 静态结果等于 computeGeneralizedGravity。夹爪模型来源为本地官方 agx_arm_urdf，包含夹爪法兰和双指，双指冻结于模型零位，无相机。实际夹爪是否与该型号一致尚未核实。

## 尚未验证

上次 leader 脚本摆动根因、从当前实物姿态使能后平稳接入补偿、实际夹爪型号/质量匹配均未验证。没有已验证的无突变启动命令。原版中断与异常分支会 move_j(最近一轮读取角度)，不能把退出当成被动断开。

官方讨论 #44 描述了旧版本 NERO 与夹爪组合的突动，但版本和调用路径与当前不同，不能当成本次原因：https://github.com/agilexrobotics/pyAgxArm/discussions/44

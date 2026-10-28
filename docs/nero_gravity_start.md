# 按官方示例启动 NERO 双臂/单臂重力补偿

2026-10-26：启动器默认启动双臂，左臂 can1、右臂 can0；标准夹爪，无相机。
先退出 MoveIt/其他控制这两只机械臂的程序，保持 CAN 已激活。

```zsh
# 默认双臂
bash /home/a/Nero-7/scripts/start_nero_gravity.sh
# 仅左臂
bash /home/a/Nero-7/scripts/start_nero_gravity.sh left
# 仅右臂
bash /home/a/Nero-7/scripts/start_nero_gravity.sh right
```

也支持 `--arm left` / `--arm right` / `--arm both`；旧 `--execute` 仍兼容，默认是双臂。
双臂各运行一份官方配置示例，左侧 RPY=(-1.57,-1.57,0)、右侧 RPY=(1.57,-1.57,0) rad。
Ctrl+C 同时通知两个进程执行官方中断退出行为。任一进程退出，启动器也结束另一进程。
没有增加自定义使能检查、控制门控或自动复位。
五项离线配置/进程测试通过；本次没有运行实物补偿。

以下为之前恢复官方流程的记录（旧的默认仅右臂已由上面的默认双臂替代）。

2026-10-25：按用户要求撤回自定义检查、反馈等待状态机和复位入口。

来源为 `third_party/agilex-arm-gravity-compensation` 的固定提交
`8e2545c00ec9ec7590414946ab5f913274c7b643` 中 `nero/main_gc.py`。
配置副本为 `scripts/nero_gc_official_configured.py`，接入差异：

1. 官方 Python 模块路径适配本工程。
2. URDF 改用已有标准夹爪、无相机模型。
3. 根据 `--side` 选择左右 CAN 与重力安装 RPY；直接调用 Python 文件不传参数时仍默认为右臂。

固件保留官方 `NeroFW.V121`，200 Hz 目标周期、使能、
首帧等待、动力学计算、MIT 下发、异常和 Ctrl+C 行为均保持固定版本官方代码。
Ctrl+C 按官方行为发送最近读到的关节角度 move_j；不保证退出时绝对静止。

```zsh
bash /home/a/Nero-7/scripts/start_nero_gravity.sh
```

也兼容原有 `--execute`。现在默认直接连接并使能，不再默认离线检查。
旧 `--feedback-check` / `--reset-estop` 等参数由 shell 拒绝，避免误调用后进入实机控制。

已验证：配置副本反向撤回三处路径/安装角修改后，与固定版本官方源码逐字一致；
Python 语法、shell 语法及帮助/无效参数行为正常。本次没有启动实物或复位急停。
原自定义版本仅备份于 `log/gravity_environment/custom_before_official_restore.py.txt`。

先前观察到的电子急停是控制器状态，恢复官方代码本身不会清除它。
本地官方 SDK 文档 `third_party/pyAgxArm/docs/nero/nero_api.md` 的 Reset 章节说明：
reset 会立即失能，悬空机械臂可能下落，需要现场托稳后按官方流程处理；启动入口不自动复位。

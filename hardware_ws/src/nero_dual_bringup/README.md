# 一个 RViz 控制两只 NERO

默认：左臂 can1、右臂 can0，双臂均为标准夹爪、无相机。

退出原单臂 MoveIt 和重力补偿程序，确认两个 CAN 口已经接好、激活：

```zsh
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
bash /home/a/Nero-7/scripts/start_dual_nero_moveit.sh
```

可覆盖端口：`left_can:=can1 right_can:=can0`。启动默认按官方行为自动使能两臂。
RViz 当前姿态来自真实反馈；Planning Group 可选 `left_arm`、`right_arm`、
`both_arms`、`left_gripper`、`right_gripper`。设置目标后 Plan & Execute。

`left_arm`、`right_arm` 和 `both_arms` 的 Goal State 均提供 `park`：
每臂 J2=85°、J4=5°，其余关节为 0；夹爪不属于此姿态。
单臂官方 MoveIt 的 `arm` 组也已加入同一预设。这是本地增加的命名姿态，未修改控制流程。

独立回 park（不需要 MoveIt/RViz；先退出其他控制程序）：

```zsh
bash /home/a/Nero-7/scripts/park.sh        # 双臂
bash /home/a/Nero-7/scripts/park.sh left   # 仅左臂
bash /home/a/Nero-7/scripts/park.sh right  # 仅右臂
```

当前 `park.sh` 直接使用官方 SDK `move_j`，速度比例 10%，
目标固定为每臂 [0,85,0,5,0,0,0] 度；等待反馈角度各轴误差≤0.5°才报告到位。
不启动 ROS、MoveIt、RViz，不自动失能或断电；此路径不包含 MoveIt 碰撞规划。
Ctrl+C 按当前关节反馈发送保持目标。默认双臂，left=can1、right=can0。
SDK 调用映射测试通过，尚未替用户执行实物 park 或验证断电支撑。
此前依赖 MoveIt 的版本仅保留为 `scripts/move_to_park_moveit.py`，不再作为 park.sh 的入口。

独立失能（先退出 MoveIt、重力补偿等控制程序，CAN 保持连接）：

```zsh
bash /home/a/Nero-7/scripts/disable.sh        # 双臂
bash /home/a/Nero-7/scripts/disable.sh left   # 仅左臂
bash /home/a/Nero-7/scripts/disable.sh right  # 仅右臂
```

读取完整且不超过 0.5 秒的真实七轴反馈；每轴相对 park 的偏差必须≤3°。
也就是 J2 在 [82°,88°]、J4 在 [2°,8°]、其余在 [-3°,3°]。
所选机械臂全部通过后才调用官方 SDK disable，随后要求七轴的新使能反馈均为 False 才报告失能确认。
不会自动回 park、复位或断电。位置检查使用固定 SDK 的角度分组时间戳，避免缺失轴被 SDK 聚合成零。
六项模拟测试通过，覆盖某一臂不在 park 时两臂均不发失能、其他轴超差、反馈缺失/过期等情况。
本次没有执行实物失能；角度条件不是机械支撑效果的验证。

## 来源与改动边界

- 官方 `agx_arm_ros` 的驱动、SDK、单臂 URDF/SRDF、关节限制与控制门程序均未修改。
- 将官方 MoveIt 单臂 profile 展开两次，加 `left_` / `right_` 前缀；保留夹爪总宽度关节及其官方 mimic 关系。
- 底座几何及安装变换来自官方 `open_nero` 提交 `9164ce5b4ced1f0926a18849fc6cf70b392a4e54`。
  只取底座和左右安装关节，不添加相机和顶部相机支架。
- `model.py` 组合模型、SRDF 和四个控制器，增加 `both_arms` 子组组合；两臂之间仍执行碰撞检查。
- `bridge.py` 仅对关节名加/去前缀，真实反馈送入共同 `/joint_states`；命令送到官方 `/left|right/control/joint_states`。
  同时按正在执行的控制器筛选关节，避免夹爪执行时夹带空闲机械臂的 mock 初值。
- 每臂使用原样的官方 `agx_arm_control_gate`，启动控制门关闭，执行后关闭。
- 没有新 IK、CAN 协议、重力补偿或自动回零流程。此包是本地双臂接线，并非厂商发布的双臂实机包。

## 验证（2026-10-25）

构建五个包成功，启动配置解析通过；六项接线/资源测试通过。
隔离 ROS_DOMAIN_ID=93，不启动任何 CAN 驱动，运行真实 MoveIt、mock ros2_control、映射节点及官方门控：

- 16 个关节/夹爪反馈名称到达共同 joint_states；
- 启动期间向驱动命令话题发送数为 0；
- 左臂轨迹产生 210 帧左臂命令，不向右臂发送；
- 右夹爪轨迹产生 210 帧夹爪命令，不夹带机械臂目标；
- 执行结束官方门控关闭；
- left_arm / right_arm / both_arms 的 MoveIt 规划均返回 SUCCESS（1）。
- both_arms 经 MoveIt ExecuteTrajectory 联合执行，两个控制器均完成；左右各转发 30 帧模拟命令，结束后门控关闭。

报告：`hardware_ws/validation/dual_moveit/mock_report.json`，日志同目录。
当前系统没有 can0/can1，未做双臂实机执行验收。以上不代表碰撞模型与实物安装误差已标定，也不代表实物轨迹跟踪已验证。
# 柜体与安装高度

原有 `bash /home/a/Nero-7/scripts/start_dual_nero_moveit.sh` 现在默认加入前方 60 cm 高柜体和地面，固定支撑底部离地 28 cm。可用 `support_height`、`cabinet_height`、`cabinet_width`、`cabinet_depth`、`cabinet_front` launch 参数调整，单位均为米。柜宽 1.2 m、深 0.4 m、近侧 X=0.50 m（按实物指尖越过柜前沿 3.5 cm 校准；宽深仍待实测）；RViz 固定帧为 world。柜体与地面保留机械臂碰撞检查，仅固定支撑与中间底座的安装接触被允许。

官方驱动、关节桥接和轨迹执行逻辑保持原样。实物启动不自动发送 park，RViz 的 park 沿用原有预设。细节见 `/home/a/Nero-7/docs/cabinet_sim.md`。

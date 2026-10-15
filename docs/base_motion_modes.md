# C++ 底盘纯平移、纯转向与组合运动

现有 `base_move_to` 保留原来的 Python 分段路线控制，水瓶任务 JSON 未修改。
新增三种动作通过 `/task_motion/base` 的 `ExecuteBaseMotion` Action 交给独立 C++ 节点。

| 类型 | 必需目标字段 | 保持的量 |
| --- | --- | --- |
| `base_translate` | `frame_id`, `position_xy_m` | 动作开始时反馈的朝向 |
| `base_rotate` | `frame_id`, `yaw_rad` | 动作开始时反馈的位置 |
| `base_move_and_rotate` | `frame_id`, `position_xy_m`, `yaw_rad` | 位置和朝向沿共同进度变化 |

三个类型只接受 `odom` 坐标，目标是绝对目标，角度单位 rad。纯平移不能附加 yaw 目标，纯转向不能附加 xy 目标，避免动作含义不清楚。保持某个量允许反馈校正：纯转向发现位置漂移时可以发送纠偏平移速度，并不意味着强制 vx/vy 永远等于零。

参考点是在服务器接受目标并开始动作时，从新鲜里程计捕获的位置和朝向。新类型不接受 `via`；要多段运动，按顺序写多个动作。保留的 `base_move_to` 仍支持原途经点。

## 示例

`sim_ws/src/nero_task_executor/examples/base_modes.json`：

1. 保持初始朝向，平移到 (0.4,0)。
2. 保持当前位置，转到 pi/2。
3. 从当前位置开始，边平移边转向，回到 (0,0,0)。

此示例要求从重置的场景开始，三路中的左右臂列表为空。相同 task_id 不会重复执行。

完整夹水搬运的组合版本为 `sim_ws/src/nero_task_executor/examples/bottle_transfer_combined.json`（同名 `.jsonc` 有注释）：源桌靠近保持朝向平移；抬瓶完成后，向左桌移动并同步转 180°；放瓶完成后，向原点移动并同步恢复朝向。原 `bottle_transfer.json` 仍使用分段路线。新任务须从重置的默认场景运行，机械臂目标与原任务一致。

构建：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select nero_task_interfaces nero_task_executor
source install/setup.zsh
```

先按根目录 README 的三个终端启动场景和 `simulation.launch.py`，第三个终端发送：

```zsh
source /opt/ros/humble/setup.zsh
source /home/a/Nero-7/sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=75
ros2 run nero_task_executor submit_task \
  /home/a/Nero-7/sim_ws/src/nero_task_executor/examples/base_modes.json
```

新启动的 `simulation.launch.py` 会自动启动 `sim_base_server`。旧进程不会自动加载新节点；修改接口后先停止旧 launch，再重启完整场景。不要同时运行键盘控制等其他 /cmd_vel 指令源。

## 可选参数

这些字段放在每个新动作的 `parameters` 内，三个模式共用：

| 字段 | 默认值 | 适配器允许上限 |
| --- | --- | --- |
| `max_linear_speed_m_s` | 0.10 | 0.20 m/s |
| `max_angular_speed_rad_s` | 0.18 | 0.40 rad/s |
| `linear_acceleration_m_s2` | 0.15 | 0.40 m/s² |
| `angular_acceleration_rad_s2` | 0.30 | 0.80 rad/s² |
| `duration_s` | 自动计算 | 必须短于动作 `timeout_s`，且足够满足参考轨迹的速度、加速度约束 |

`duration_s` 是参考轨迹持续时间，实际完成还要等待到位和停止反馈。太短的参考时间会被服务器拒绝，不会偷偷突破限速。朝向采用最短角差插值，180° 附近的方向由捕获到的起始朝向决定；不支持指定整圈旋转。

## 共用控制核心

- `sim_ws/src/nero_task_executor/include/nero_task_executor/base_pose_controller.hpp`：不依赖 ROS 的参考轨迹、反馈和限幅。
- `sim_ws/src/nero_task_executor/src/sim_base_server.cpp`：ROS Action、里程计、计时、停止与结果。
- `sim_ws/src/nero_task_executor/nero_task_executor/sim_backend.py`：异步派发与底盘控制权交接。

令起始位姿为 q0，目标为 q1，u=clamp(t/T,0,1)，共同进度为：

```text
s(u) = 10u³ - 15u⁴ + 6u⁵
q_ref = q0 + s(u) * (q1-q0)
v_ref = ds/dt * (q1-q0)
```

朝向差先包裹到 [-pi,pi]。三个模式共用公式；纯平移把朝向差设为零，纯转向把位置差设为零。

参考速度加上位姿误差反馈后，世界坐标速度经 R(-yaw) 转成底盘坐标的 vx/vy，并与 wz 一起发布。组合模式没有“朝向到位才允许平移”的门槛。

参考曲线起止速度为零，持续时间按速度、加速度约束推导，给反馈速度留 30% 余量。模型存在低速停滞，因此仅在参考速度接近零时按权重连续加入最低驱动补偿，巡航阶段逐渐退出该补偿；在参考停止且误差超出内层到位容差时保留完整低速驱动力。补偿保持前馈加反馈后的速度方向，并在合成速度经过零时平滑退去。命令再经过速度限幅和加速度限幅。补偿会改变实际运动，不能把平滑参考等同于物理轨迹严格同步或电机硬实时控制。

## 完成、停止与控制权

参考走完且误差进入 12 mm / 0.015 rad 后发送零速度；制动允许 25 mm / 0.03 rad 的较宽区间，超出时重新修正。最终还要确认轮式反馈与位姿差分速度均较低，持续 0.4 s。

取消或超时发送零速度并等待停止。里程计接收时间、时间戳推进和仿真时间均检查有效期；反馈失效时不会凭零指令报告已停。停止未确认会锁定服务器和上层调度器，拒绝新任务。连续 12 s 没有有效位置/朝向进展时报告 BASE_STALLED。

C++ 服务器空闲时不发布速度。启动新动作时 Python 停用旧底盘保持；新动作终止并确认停止后，Python 才接回停靠保持。旧 `base_move_to` 与新动作不能同时占用底盘。

这是当前 Gazebo 模型的位姿跟踪能力，没有障碍规划，也没有联合机械臂动态避碰。携瓶运动要另外验证夹爪与物体的扫掠空间；本次不替换已经验证的水瓶搬运路线。

## 已验证结果

45 项单元测试和六项 DDS 协议检查通过。Gazebo 实际测得组合动作中平移与转向重叠约 9.9 s；运动中取消确认停车且阻断后续动作。原水瓶搬运流程完整回归成功，任务 JSON 未修改。详细测量与证据范围见 [验证记录](../sim_ws/validation/base_modes/README.md)。

## 组合运动平顺性

底层 AIR 控制器按各轮所需速度加权转向误差，并使用连续的对齐减速曲线。靠近瞬时旋转中心、所需轮速很低的轮子不会仅因转向变化快而拖停整车；有效误差较大时仍停轮对齐。上层低速补偿也采用连续变化，并保留合成速度方向。取消与反馈超时仍直接进入停止流程。测试与对照数据见 [平顺性诊断记录](../sim_ws/validation/base_smoothness/README.md)。

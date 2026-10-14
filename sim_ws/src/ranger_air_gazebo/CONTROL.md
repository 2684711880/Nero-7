# AIR 移动控制

## 启动

终端 1（zsh；先用 Ctrl-C 结束同一 Gazebo 端口的其他仿真）：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch ranger_air_gazebo control.launch.py
```

终端 2：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run ranger_air_gazebo air_keyboard
```

W/S 前进、后退；A/D 左右横移；Q/E 逆时针、顺时针旋转；空格停车；Ctrl-C 退出。按下运动键后持续运动，不必一直按住；切换按键会更新运动指令。保持键盘终端打开，并在退出前停车。默认平移 0.15 m/s、旋转 0.4 rad/s。

```zsh
ros2 run ranger_air_gazebo air_keyboard --ros-args -p speed:=0.1 -p turn:=0.3
```

终端 1 按 Ctrl-C 退出整套仿真。仅关闭 Gazebo 窗口可能留下后台服务。

需要与双臂仿真同时独立运行时，在 AIR 的所有终端中，加载环境后、运行命令前加入：

```zsh
export ROS_DOMAIN_ID=37
export GAZEBO_MASTER_URI=http://127.0.0.1:11355
```

这用于分别测试；后续整车组合时需统一模型和 ROS 域。没有设置 ROS_DOMAIN_ID 的普通终端无法控制域 37 内的小车。

## 控制与反馈

`/cmd_vel (Twist)` → `air_controller`（50 Hz）→ `/steering_controller/commands`（4 个关节位置）、`/wheel_controller/commands`（4 个轮速）→ `gazebo_ros2_control`（100 Hz）→ Gazebo 物理（1 kHz）。

四轮顺序统一为 FL、FR、RL、RR；模型转向符号为 `+,+,+,-`，驱动符号为 `-,-,+,-`。转向轴位置为 `(±0.194, ±0.169)` m，轮胎半径为 0.0605 m，均来自模型，不是实车标定。

转向使用最短路径，必要时反转轮速；转向命令限速 1.5 rad/s。任一轮目标转角与反馈误差超过 0.12 rad 时暂时停止驱动，等转向对齐。零速度保持转向。平移限速 0.4 m/s、旋转限速 0.8 rad/s；加速度限制分别为 0.4 m/s² 和 0.8 rad/s²。最后一条有效命令超过 0.5 秒或关节反馈超过 0.3 秒未更新时，输出零轮速；非有限速度命令触发停止。看门狗使用墙钟，暂停恢复后旧命令会过期。控制节点退出会关闭仿真。

加减速与转向变化量按实际仿真时间间隔计算（单次最多 0.1 秒），避免组合双臂后控制回调降频时出现减速和转向变慢。`test_controller_timing.py` 在实际 timer 回调接口上验证 50 Hz 和 20 Hz 下停止时间一致。

`joint_state_broadcaster` 发布实际 `/joint_states`；无预览零位发布器。`air_controller` 使用反馈轮速与转角估算 `/odom`，发布唯一的 `odom → base_link` TF；`robot_state_publisher` 发布关节 TF。所有运动仿真节点统一使用仿真时间。

RViz 的 Fixed Frame 为 odom，相机 Orbit 的 Target Frame 为 base_link：世界参考系保留 odom，但相机会随车移动。如果手动修改相机后看不到小车，在 Views 中将 Target Frame 设回 base_link，并重置视角。

Gazebo 的 `/gazebo/model_states` 是独立验证真值，不参与控制或轮式里程计。轮式里程计依赖无侧滑近似，转向过程和打滑会产生误差；协方差为未标定的名义值。

当前保留官方 CAD 外观和惯量；运动模型将轮胎碰撞替换为半径 0.0605 m、宽 0.051 m 的圆柱，将车体碰撞简化为 0.50×0.30×0.10 m 的箱体，忽略转向支架和传感器碰撞。转向与驱动使用 GazeboSystem 理想位置/速度接口，未模拟实车电机电流、力矩饱和与闭环响应。因此当前结果只证明简化底盘在平地上的运动链可用，尚未验证传感器、双臂载荷、坡道、障碍物或真实硬件。

## 验证与记录

已有验证目录：`/home/a/Nero-7/sim_ws/validation/ranger_air/run_01/`。

- `report.json`：8 项 Gazebo 测试均通过，含运动方向、漂移、停止与里程计误差判断。
- `telemetry.csv`：命令、Gazebo 位姿、轮式里程计、四轮转角和四轮速度。
- `colcon test`：3 个运动学回归测试通过。

每项运动发送 5 秒指令，再发送 1.5 秒零速度并测量终点。直行和横移要求目标方向位移大于 0.3 m、横向漂移小于 0.08 m、航向偏差小于 0.12 rad；纯旋转要求目标方向旋转大于 0.7 rad、中心平移小于 0.10 m；组合运动要求位移大于 0.2 m、旋转大于 0.3 rad。各项轮式里程计相对 Gazebo 的位移增量误差须小于 0.12 m、航向增量误差小于 0.15 rad。停止要求轮速小于 0.1 rad/s、车体平移速度小于 0.02 m/s、旋转速度小于 0.03 rad/s。

| 动作 | Gazebo 测得位移/转角 | 结果 |
| --- | --- | --- |
| 前进 0.15 m/s | 前进 0.7094 m | 通过 |
| 后退 0.15 m/s | 后退 0.7094 m | 通过 |
| 首次左移 0.15 m/s | 左移 0.4254 m，含转轮等待 | 通过 |
| 右移 0.15 m/s | 右移 0.7169 m | 通过 |
| 逆时针 0.4 rad/s | 旋转 1.7223 rad，中心漂移约 0.72 mm | 通过 |
| 顺时针 0.4 rad/s | 旋转 -1.9117 rad，中心漂移约 0.26 mm | 通过 |
| 组合 0.1/0.05 m/s、0.2 rad/s | 车体初始坐标下移动 0.2373/0.2261 m，旋转 0.5895 rad | 通过 |
| 指令中断 | 中断 1.5 秒后速度约 1.46e-6 m/s | 通过 |

加速、转向等待和动作初始轮组方向不同，会导致同样 5 秒命令的位移不相等。当前测试是运动链验收，并非高精度轨迹跟踪验收。

重新验证（需要已启动移动仿真，关闭键盘及其他 `/cmd_vel` 发布者）：

```zsh
ros2 run ranger_air_gazebo verify_motion \
  --output-dir /home/a/Nero-7/sim_ws/validation/ranger_air/new_run \
  --ros-args -p use_sim_time:=true
```

仿真独立运行在域 37 时，验证终端也需设置对应 ROS_DOMAIN_ID。验证结束会发送零速度，并保存报告；失败会返回非零退出码。

```zsh
cd /home/a/Nero-7/sim_ws
colcon test --packages-select ranger_air_gazebo
colcon test-result --test-result-base build/ranger_air_gazebo --verbose
```

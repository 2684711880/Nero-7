# 从代码画直线：RViz 预览后执行

主文件：`hardware_ws/src/nero_official_commander_cpp/src/straight_line.cpp`。保留现有 commander，新增独立可执行程序，不改 CAN 驱动、joint_bridge 或控制器。

## 实物使用

先退出重力补偿或占用同一机械臂的其他控制程序，按已有方式启动：

```zsh
bash /home/a/Nero-7/scripts/can.sh
bash /home/a/Nero-7/scripts/start_dual_nero_moveit.sh
```

保持 RViz 运行，在另一个终端运行（先预览，等待输入）：

```zsh
bash /home/a/Nero-7/scripts/straight_line.sh
```

默认左臂，读取当前关节反馈，经 FK 取得 TCP 起点 A；B=A+(0.02,0,0) m，保持朝向。规划完整的 2 cm 直线并发送 RViz 动画，尚不发送执行请求。程序保持运行，输入小写 `do` 才执行；Enter 或其他输入取消。RViz 的 MotionPlanning → Planned Path → Trajectory Topic 为 `/display_planned_path`，可开启 Loop Animation。

正常命令不需要 `-p execute:=true`。右臂向前 10 cm：

```zsh
bash /home/a/Nero-7/scripts/straight_line.sh --ros-args -p arm:=right -p dx:=0.10
```

查看本次 RViz 预览，然后在终端输入小写 `do`。其他输入或 EOF 取消。确认后的执行使用内存中刚刚预览的轨迹，不重新规划。确认期间反馈过期、机器人起点变化或规划场景变化则拒绝执行，需重新运行并预览。

只想预览、即使输入 do 也不执行，可显式设置：

```zsh
bash /home/a/Nero-7/scripts/straight_line.sh --ros-args -p execute:=false
```

## 后续修改

源文件顶部集中放置 ARM、FRAME、DX/DY/DZ、SPEED、STEP。最常改的是 DX/DY/DZ：

```cpp
constexpr double DX = 0.02;  // 世界 +X，前方 2 cm
constexpr double DY = 0.00;  // 世界 +Y，左方
constexpr double DZ = 0.00;  // 世界 +Z，上方
```

位移单位 m，不是末端自身坐标。SPEED=0.10 是关节速度/加速度上限的 10%，不是 0.1 m/s。TCP 由当前规划组模型确定，实物组为 `left_tcp_link` / `right_tcp_link`；不是自动测量的夹爪尖端或工具杆尖。

无需编译也可临时覆盖方向和机械臂：

```zsh
# 左臂向上 2 cm，预览后输入 do 执行
bash /home/a/Nero-7/scripts/straight_line.sh --ros-args -p dx:=0.0 -p dz:=0.02

# 右臂向前 1 cm，预览后输入 do 执行
bash /home/a/Nero-7/scripts/straight_line.sh --ros-args -p arm:=right -p dx:=0.01
```

改 C++ 后编译：

```zsh
cd /home/a/Nero-7/hardware_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select nero_official_commander_cpp
```

运行程序即可使用新代码，双臂实物驱动不用因修改位移而重启。

## 实现和边界

流程：当前真实反馈 → FK 起点 → 固定朝向的笛卡尔直线插值（1 mm 步长，碰撞检查开启）→ 完整路径检查 → 时间参数化 → RViz DisplayTrajectory → 人工确认 → 原有 MoveIt ExecuteTrajectory → 原有控制器/官方驱动 → 反馈确认终点。

直线规划不完整时直接退出，不执行部分路径，不自动改起点或先回 park。重复执行会以新的当前位置为起点，位移会累加。当前姿态、关节限位、奇异性或障碍物都可能使某个方向的直线不可行。

执行时持续保证柜体、安装高度、工具尺寸与现场一致。RViz 动画是规划几何预览，没有运行 Gazebo 动力学；模型碰撞通过也不是实物安全保证。Ctrl+C 不能替代现场急停。

## 本次验证

2026-10-27 已编译，使用 ROS domain 93 的无 CAN 模拟驱动验证：从 park 模拟反馈沿世界 +X 规划 2 cm，RViz 预览轨迹 3 点、约 0.361 s；仅预览阶段 0 条运动指令。输入 do 后走原有 MoveIt/控制器/桥接链路，左臂收到 80 帧、右臂 0 帧，模拟终点反馈最大误差 0 rad。合成反馈零误差不代表实物精度。本次没有操作实物，也没有进行 Gazebo 动力学测试。

报告：`hardware_ws/validation/dual_moveit/straight_line_report.json`；测试：`hardware_ws/src/nero_official_commander_cpp/test/verify_straight_line.py`。

## 代码执行后 RViz 再执行时报起点不一致

2026-10-27 实物日志记录：右臂 10 cm 直线执行成功，并由代码的关节反馈检查确认终点。之后 RViz 单独发起的执行被拒绝：`Invalid Trajectory: start point deviates from current robot state more than 0.01`，`right_joint1` 轨迹起点为 0.00542797 rad，当前为 0.223961 rad，差约 12.52°。

这是后续 RViz 请求使用了与实物不一致的起点。代码动作结束后，在 RViz 将 Start State 重新选为 `<current>`，重新 Plan，再执行新轨迹。不要执行移动前的旧轨迹，也不要通过放宽 `allowed_start_tolerance` 绕过检查。此记录没有通过再次操作实物复现，结论来自同一次执行的现有日志。

已回归验证新交互：普通命令预览后 Enter 取消时 0 条运动指令；普通命令输入 do 后仅所选左臂执行，右臂 0 帧。测试使用独立 domain 93 的无 CAN 模拟反馈，本次未操作实物。

## 执行后自动对齐黄色目标模型

已启用 RViz 自带 `MoveIt_Allow_External_Program` 配置。直线程序在执行成功且反馈确认终点之后，向官方 `/rviz/moveit/update_start_state` 和 `/rviz/moveit/update_goal_state` 各发一个 Empty 请求，刷新查询起点及黄色目标模型；不发送 RViz plan/execute 请求。预览/取消时不刷新，避免覆盖手动选好的目标。未开启 External Comm. 时程序会明确提示，但不影响已经完成的运动。

当前运行的 RViz 需手动勾选 Planning → External Comm.；下次启动默认开启。外部代码执行后仍需为下一个任务重新 Plan，不执行移动前生成的旧轨迹。黄色目标在手动拖动规划时仍代表你设置的目标，不会被持续强制跟随。

2026-10-27 现场仅刷新 UI 验证：右臂黄色末端 marker 的 X 从 0.03152955 m 变为 0.23148051 m，约 20 cm；与反馈 FK 的位置差约 0.00004205 m。这是同一模型中的数值对齐，不是实物绝对精度测量。见 `hardware_ws/validation/dual_moveit/rviz_query_refresh_live.json`。模拟回归确认预览 0 条运动指令，执行后 start/goal 各刷新 1 次。

官方实现： https://github.com/moveit/moveit2/blob/humble/moveit_ros/visualization/motion_planning_rviz_plugin/src/motion_planning_frame_planning.cpp （remoteUpdateStartStateCallback / remoteUpdateGoalStateCallback）。

## 在同一份代码上画圈

右臂半径 3 cm、直径 6 cm 的完整圆：

```zsh
bash /home/a/Nero-7/scripts/straight_line.sh --ros-args \
  -p arm:=right -p shape:=circle -p radius:=0.03 -p plane:=xy
```

先预览，输入 do 执行。`plane` 可选 xy（水平）、xz（前后-上下）、yz（左右-上下），都相对 world。`radius` 单位 m，0.05 表示半径 5 cm。圆模式忽略 dx/dy/dz；不传 shape 时仍画直线。

当前 TCP 位置 A 是圆上的起点，保持姿态，不额外运动到其他起点；XY/XZ 圆心在 A 的 +X 方向 radius 处，YZ 圆心在 A 的 +Y 方向 radius 处。圆在第一轴方向覆盖 0…2r，另一轴覆盖 -r…+r，最后回到 A。不能把 A 当作圆心。

源码仍是 `straight_line.cpp`：顶部 SHAPE/RADIUS/PLANE 为默认值，`waypoints` 分支生成圆上的采样点。整圈统一求笛卡尔路径、统一时间参数化，执行一个轨迹；不是分两段圆弧。时间参数化限制的是关节速度/加速度，不保证末端沿圆恒速。路径不完整则不执行部分圆。

圆模式已编译并通过无 CAN 模拟反馈验证：预览 0 条运动命令，执行仅右臂收到指令，终点反馈确认；模型 FK 验证轨迹采样点到圆的径向偏差约 4.74 微米、闭合位置差约 1.76 微米。这些是模型内部计算误差，不是实物精度，也未验证插值采样点之间的实际轨迹。报告见 `hardware_ws/validation/dual_moveit/circle_report.json`。本次未操作实物。

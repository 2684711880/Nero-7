# 双臂深度孔穿入仿真

动作入口仅连接 ROS domain 93 的 Gazebo 仿真，**不读取相机 USB、不打开 CAN、不控制实物**。启动前在当前 ROS domain 等待最多 2 秒，监听 `/right/feedback/joint_states`；收到七轴反馈就作为仿真右臂初值，否则使用模型的右臂 park。请确认该话题由实机反馈节点发布：话题本身不能证明其来源，也不会自动启动实机反馈节点。环境与构建见 [demo README](../demos/hole/README.md)，检测器已经纳入本仓库。

## 启动和使用

先关闭其他 Gazebo 实例，然后在第一个终端：

```bash
# 在仓库根目录运行
bash scripts/sim_hole.sh start
```

默认打开 Gazebo 和 RViz。底部有 28 cm 垫高，左夹爪从 40×40 cm 方板的侧边夹持，两个手指分别位于板的正反面，板边伸入夹爪约 50 mm；孔在板中心，名义直径 90 mm。夹爪和板使用固定约束，不模拟摩擦夹紧。右杆直径 16 mm，伸出模型夹持中心 40 mm。调整仿真几何需改 `sim_ws/src/dual_nero_moveit_config/launch/hole_scene.py` 并重新构建 `dual_nero_moveit_config`。

在 RViz 的 MotionPlanning 面板选择 `left_arm`，拖动末端目标，使用 `Plan` 和 `Execute` 移动左臂与板。目标必须在规划可达范围内。右臂固定粗观察位只针对默认板位附近，左臂移动过大可能看不到孔；此时需要重新摆板或手动调整右臂观察位。

彩色画面可另开终端运行 `bash scripts/sim_hole.sh camera`，深度识别画面使用 `bash scripts/sim_hole.sh detect`。

右臂初始不在观察位。先规划到一个预设的粗略观察关节姿态（不读取板或孔的真值）：

```bash
# 在仓库根目录运行
bash scripts/sim_hole.sh observe-plan
```

观察 RViz 预览后，执行这段仿真轨迹：

```bash
bash scripts/sim_hole.sh observe-execute
```

在第二个终端看深度识别（`q` 退出）：

```bash
# 在仓库根目录运行
bash scripts/sim_hole.sh detect
```

到达观察位后，另开终端，检测最新深度帧并规划，结果在 RViz 显示；不发送轨迹：

```bash
# 在仓库根目录运行
bash scripts/sim_hole.sh plan
```

确认模型与路径后，在仿真中执行保存的两段轨迹：

```bash
# 在仓库根目录运行
bash scripts/sim_hole.sh execute
```

穿孔完成后，右臂沿本轮轨迹反向退出孔、退回观察位，再返回观察前的右臂位置，便于重新摆左臂：

```bash
bash scripts/sim_hole.sh retreat --check  # 只检查，不移动
bash scripts/sim_hole.sh retreat          # 执行仿真回退
```

回退要求本轮观察和穿孔都已执行成功，且左臂仍在穿孔时的位置、右臂仍在穿孔终点；回退前会对路径进行碰撞检查。左臂重新摆位后，需要重新运行观察、检测和规划。

执行入口检查 Gazebo 硬件插件、仿真时钟、专用模型和双臂起始状态。两段动作分别执行：先将杆尖对准孔前 20 cm，再沿孔法向连续直线前送 23 cm。两段之间是动作接口交接，不能声称交接速度连续。需要再演示一次时先 Ctrl+C 关闭仿真，再重新启动。也可用 `bash scripts/sim_hole.sh stop` 请求关闭这个场景；深度窗口按 `q` 关闭。

可用 `bash scripts/sim_hole.sh start board_j1_offset:=0.03` 在启动时改变左臂持板位置；右臂粗观察位仍为预设，孔中心和法向必须由新深度帧检测。测试范围为 ±0.10 rad，并不保证所有位置都在视场或可达。

## 数据与边界

`sim_ws/validation/hole_approach/latest/` 保存深度识别图、孔参数、规划与执行结果；`square_default/` 和 `square_offset_j1_003/` 分别保存方板默认位置与左臂 J1 偏移 0.03 rad 的记录。旧圆板 `first_loop/` 仅留本地，不随当前方板 demo 发布。历史目录只上传检测图、检测参数和执行结果，轨迹计划保留本地。检测调用仓库内 `scripts/depth_hole.py`（ArmHub 快照），不依赖其他机器人驱动。相机位置按右腕 `T_gripper_camera` 变换设置；虚拟深度和彩色光学中心视为重合，虚拟相机的视场与深度均为理想模型，不等同于真实 D405 的噪声、遮挡或深度到彩色外参。

检测 JSON 的 `center_error_mm` 比较检测到的可见板表面中心与模型板中面中心；板厚 8 mm，因此该指标含约 4 mm 的面到中面偏移，不应称为纯横向检测误差。执行 JSON 中 `radial_error_mm` 与 `signed_plane_offset_mm` 是仿真杆尖相对板中面的结果。右腕手眼与实际杆尖 TCP 均未经过独立实机定位验收。

模拟摄像机和杆只用于该实验，不能默认复用其他场景的虚拟 D435 安装位置。双臂动力学、固定夹持、碰撞几何和相机外参均为仿真模型；成功穿孔不代表实物能够直接执行。Gazebo/MoveIt 在部分环境关闭时可能打印析构阶段段错误，与运行期结果分开记录。

# can1 D405 手眼结果与左上角指点预览

## RViz 显示、轨迹预览、显式执行

RViz 显示完整双臂与躯干，使用官方左右侧装位姿。左臂 `can1` 执行本次指点，
右臂 `can0` 只读取反馈，并参加碰撞检查，不注册右臂轨迹控制器。
右臂接口可以显式用 `--other-can can0` 配置；两条臂均需有新鲜反馈。
手眼仍在左臂局部基座中定义，目标换算会先乘躯干到左臂基座的安装变换。

当前目标按用户确认使用**模型夹爪中心**，默认相对标定法兰 `[0,0,141.3] mm`。
该值来自当前 URDF：gripper_base 到法兰 5.5 mm，加两指模型原点的中心距离 135.8 mm。
这是模型定义，实物开合/指尖形状可能不同；可以通过参数覆盖。

先关闭原来的手眼采样 RViz（它占用同一台相机），保留你自己启动的重力补偿。
然后：

```bash
cd /home/a/Nero-7
bash scripts/run_pointing.sh --can can1
```

本入口自己启动相机输入、双臂模型、MoveIt、左臂官方 ROS 控制链和 RViz，不再另开 run_handeye.sh。
启动不自动使能；官方控制门默认关闭，只在显式执行轨迹期间打开。
启动前先检查 D405 是否枚举成功，缺相机时直接给出简短提示，不启动整套 ROS 栈。
本入口为 CycloneDDS 单独设置可用 participant 索引到 63（本机 Humble 默认 9 不足以容纳真实栈），
不修改系统全局环境。ROS 节点的相机参数解析兼容 `--ros-args`。
完整节点输出保存在本次目录 `console.log`，便于从第一个失败定位，而非只看连带退出报错。
相机连续 3 秒没有彩色帧时，采集端会关闭旧 SDK 流并按原序列号重新连接；
拔出期间每隔至少 5 秒重试。USB 重新协商后会重新读取实际模式、内参和去畸变映射。
重连只恢复相机采集，不触发规划或执行。启动器退出会回收自身独立进程组，释放相机。

RViz 用左上角 **Interact** 工具，右键红球菜单：

`Live camera` 窗口始终订阅 `/handeye/color/image_raw` 实时彩色流；
`/pointing/target_image` 是按下识别规划后生成的标注图，不是实时相机话题。

1. **1 Observe board and plan**：板固定、机械臂停稳后点此项，识别整板左上角并规划。
2. 观察蓝色板边框、红色角点、绿色目标夹爪中心、橙色当前模型中心；虚影机械臂播放规划。
3. **2 Replay saved trajectory**：重复播放同一条规划。
4. 你自己在重力补偿终端按 Ctrl+C 退出补偿，避免同时写 MIT 力矩和位置轨迹。
5. 确认起始姿态和板保持不变，右键 **3 EXECUTE saved trajectory**。
6. 需要取消时右键 **4 Cancel execution**。这是轨迹取消请求，不代替硬件急停。

默认夹爪中心停在角点沿板面朝相机方向 **10 cm** 处，保持当前法兰朝向。
可以明确指定模型偏移和距离：

```bash
bash scripts/run_pointing.sh --can can1 --tcp-offset-mm 0 0 141.3 --standoff 0.10
```

数据写入 `calibration_data/rviz_point_.../`：目标标注图、规划模型、
`planned_target.json` 中的关节轨迹，以及实际执行后才产生的 `execution.json`。
预览与执行使用同一份 MoveIt RobotTrajectory，执行复用官方
ExecuteTrajectory → FollowJointTrajectory → ROS joint_states → SDK 的流程，最终额外检查七轴实测反馈。

规划检查双臂、躯干与观测板面的碰撞。未包含桌面、相机支架等完整实物环境，
需要现场核对空间。红角点使用你这块竖版印刷板（横向 5 格、纵向 7 格）的左上角，
对应 OpenCV 板坐标 `[0,0,0]`。此前误用了横版的 `[0,0.20,0]`，在这块板上是右上角。
板旋转后这个固定物理角未必在图像左上，请检查标注。

离线模式（合成目标、假反馈，菜单拒绝实机执行）：

```bash
bash scripts/run_pointing.sh --can can1 --mock
```

关闭 RViz 会结束本次启动的栈；也可以在启动终端 Ctrl+C。测试模式须退出后再启动真实模式。

当前按用户选择使用 `small2` 剔除原始第 10、11 组后的 13 组结果：
来源为 `config/handeye/small2_13_candidate.json` 和 `archive/handeye/left_d405/small2_without_samples11_10.yaml`。
生效文件为 `config/handeye/left/d405.json`，仅适用于 left / can1 机械臂、D405 `262622274345` 和当前相机安装。
变换方向 `T_gripper_camera`：彩色光学系 → `gripper_flange`，不是夹爪尖端。
平移 `[69.8633, -7.8013, 36.1237]` mm。
变换和标定板参考位姿均已按选定的 13 组样本核对，训练板位置散布 5.19 mm RMS / 9.63 mm max。
这些是训练一致性指标，尚不代表独立实机定位精度。旧配置备份位于 `config/handeye/backups/`。
已经运行的指向进程需重新启动才能读取更新后的文件；修改配置本身不会触发机械臂动作。

## 备用：仅保存目标图像的只读预览

保持 RViz 采集端运行、板可见、机械臂停稳。预览订阅现有相机话题，
不打开第二个相机流，不发送 CAN，不移动机械臂；结束后自行退出。

执行前必须明确工具尖端相对 `gripper_flange` 的 XYZ 偏移，单位 mm。
这里的偏移不是关节角，也不是相机偏移，不能把其他机械臂的 4 cm 杆偏移直接套用。

以下只适用于验证点就是法兰原点的情况；实际夹爪/杆必须换成实测偏移：

```bash
bash scripts/point_charuco.sh --can can1 --target board --tcp-offset-mm 0 0 0 --standoff 0.10
```

- `--target board`：按标定板印刷方向定义的整板外侧左上角。板旋转后未必在图像左上。
- `--target marker --marker-id 0`：指定编码方块的左上角，编码 ID 必须显式给出。
- `--standoff 0.10`：工具尖端停在目标角点沿板面朝相机方向 10 cm 处。
- 姿态保持当前法兰朝向；这个操作计算的是尖端位置，不会自动旋转工具使其轴线垂直于板面。

输出到 `calibration_data/point_preview_.../`：`target.png` 红圈标识物理目标角，
`target.json` 保存角点、TCP 目标、扣除工具偏移后的法兰目标和当前七轴。
这个备用入口不发送运动或求轨迹；上方 run_pointing.sh 才是 RViz 轨迹与执行入口。

变换链：

```text
T_base_camera = FK_7(q) × T_gripper_camera
p_base_corner = T_base_camera × T_camera_board × p_board_corner
p_base_flange_target = p_base_tcp_target − R_base_gripper × offset_gripper_tcp
```

手眼 `AX=XB` 求解不直接接收关节数量，接收的是末端和标定板位姿序列。
关节数量作用于 FK：本项目必须使用 joint1..joint7，不能套用六轴 FK。
7 轴冗余还会影响后续逆运动学的解选择，但不会把手眼外参从六自由度变成七自由度。

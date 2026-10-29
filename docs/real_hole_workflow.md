# 实机：右腕 D405 视觉戳洞

总入口：[demo README](../demos/hole/README.md)。本页是左臂已经夹板且保持静止后，仅控制右臂的流程。每段运动先在 RViz 看预览，终端输入 `do` 才执行；其他输入或 EOF 取消。

## 1. 控制栈与反馈

本机映射为左臂 `can1`、右臂 `can0`，固件 v1.21，CAN 1 Mbit/s。映射是现场配置，不应在换机器后直接照搬。右臂应已经按官方流程使能；`auto_enable:=false` 不会主动使能。不要重复启动已运行的控制栈。

新启动时，在仓库根目录打开终端 1：

```bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds_pointing.xml"
bash scripts/start_dual_nero_moveit.sh \
  left_passive:=true auto_enable:=false cabinet_height:=0.0 support_height:=0.28
```

`left_passive:=true` 只被动读取左 CAN 七轴和夹爪，不启动左臂写驱动。若左臂使用独立重力补偿，仍由原有程序管理；采集和执行要求双臂静止。此处关闭原柜体，保留 28 cm 垫高。

以下所有任务终端都先设置同一环境：

```bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds_pointing.xml"
```

显示左臂附着的近似方板（只更新规划场景，不移动）：

```bash
bash scripts/real_hole.sh board
```

板为 400 × 400 mm、厚 8 mm、孔径 90 mm，从侧边夹持。板相对夹爪的位置是近似值，最终目标不使用这个估计孔位。

## 2. 右臂到观察位

```bash
bash scripts/real_hole.sh auto-observe --aim-offset 0.08
```

从新鲜双臂反馈出发，依据左夹爪位置估计观察区域，尝试可达视角。`--aim-offset 0.08` 把瞄准点沿板面偏移 8 cm，减轻杆遮挡；不是保证可见的孔位置。检查 RViz 轨迹后输入 `do`。仅规划用 `--plan-only`。

也可显式指定粗观察位，七轴为度：

```bash
bash scripts/real_hole.sh observe --joints-deg J1 J2 J3 J4 J5 J6 J7
```

将 `J1`…`J7` 替换为实际关节目标。

## 3. 看实时画面，必要时转腕

另一个终端（只接相机，不控制机械臂）：

```bash
.venv-official/bin/python scripts/preview_d405_hole.py --serial 262422270020
```

显示原始彩色图和原生深度识别图，绿圈为识别结果，黑色为无深度。左相机单测时改用 `--serial 262622274345`，不需要连接左臂 CAN。后续实机坐标采集必须使用右腕相机和对应标定。

若孔未完整进入视野，可在任务终端调整右腕：

```bash
bash scripts/real_hole.sh wrist --delta-deg 5
# 反向：
bash scripts/real_hole.sh wrist --delta-deg -5
# 改调 J6：
bash scripts/real_hole.sh wrist --joint 6 --delta-deg 5
```

默认调整 J7；支持 J5/J6/J7，单次不超过 20°。每次预览后输入 `do`，其他关节有路径约束；转腕仍可能移动杆尖。转腕或摆板后必须重新采集孔位。

看到完整孔后，按 **`q` 关闭相机窗口**，释放 USB 设备，再继续采集。

## 4. 采集 → 生成目标 → 两段预览/执行

以下命令使用本次现场暂定的杆参数：杆尖相对法兰 `(0,0,180) mm`、轴 `+Z`、直径 `16 mm`；这些数值未实测。孔前距离 `100 mm`、穿入板面 `30 mm`。换杆或调整夹持后修改参数。

```bash
bash scripts/real_hole.sh capture &&
bash scripts/real_hole.sh targets \
  --tip-mm 0 0 180 --axis 0 0 1 --diameter-mm 16 \
  --standoff-mm 100 --penetration-mm 30 &&
bash scripts/real_hole.sh two-stage
```

`capture` 检查双臂新鲜且静止的反馈、右相机序列号、设备内外参；通过 `base → 法兰 → 彩色光学系 → 深度光学系` 转换孔中心/法向。输出到 `hardware_ws/validation/real_hole/observation.json`，不发运动。`targets` 从观测生成杆尖和法兰目标，输出同目录 `targets.json`。

`two-stage` 先规划孔前接近及法向前送。第一次输入 **`do`** 执行孔前轨迹；到位后显示前送轨迹，第二次输入 **`do`** 才前送。执行前复核起点、左臂静止和观测时效。目标生成及两段程序启动要求观测不超过 30 秒，动作确认时不超过 120 秒；超时或位置改变后重新采集规划。

只看两段完整预览，在新采集并生成目标后运行：

```bash
bash scripts/real_hole.sh two-stage --preview-only
```

该程序不发轨迹；稍后真正执行时仍需按时效要求重新采集。控制器报告成功与实物确实穿孔、工具精度是不同的验证结果。当前没有实机自动回退命令。

## 5. 预览与常见问题

- 实时机器人保持原色；规划轨迹用蓝色半透明模型。Query Start/Goal 默认不显示，避免把交互目标误认成实际位置。
- 任务与 RViz 统一订阅独立 `/nero/real_hole/display_planned_path`。启动时自动运行预览桥，只转发共享话题中起点与新鲜反馈一致的 RViz **Plan** 结果，过滤旧 park 起点。
- 结束、取消或异常后，任务会用当前双臂、夹爪和附着物替换旧显示。只预览模式保留轨迹。手动复位显示：`bash scripts/real_hole.sh reset-preview`，不移动机械臂。
- 若 `Depth: no enclosed aperture with a valid rim plane`，在实时窗口检查孔边是否完整、杆是否遮挡、孔边深度是否有效。调整视角后重新采集；这不是已经获得有效孔位。
- 如果 RViz Plan 无轨迹，检查控制栈是否已构建到最新版本、预览桥是否运行、双臂反馈是否新鲜、RViz Trajectory Topic 是否为上述独立话题。

抓包对照见 [预览问题记录](../hardware_ws/validation/real_hole/rviz_preview_issue/README.md)。完整的板/杆/相机碰撞体及实际 TCP 精度尚未验收，当前流程也不做接触力闭环控制。

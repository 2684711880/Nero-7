# 双臂前方 60 cm 柜体场景

## RViz 控制实物（实际使用入口）

复用原有双臂实物控制入口、官方 CAN 驱动、joint_bridge、控制器及轨迹执行链路。仅加入 world→base_link 安装高度和柜体/地面/支撑碰撞物；park 沿用实物配置原有的命名姿态。启动时读取实物姿态，不把仿真 park 初始角度发送给实物。

```zsh
bash /home/a/Nero-7/scripts/can.sh
bash /home/a/Nero-7/scripts/start_dual_nero_moveit.sh
```

先退出使用相同机械臂的重力补偿或其他控制程序。当前默认左 can1、右 can0，原有自动使能行为保持不变。在 RViz 选择 `left_arm`、`right_arm` 或 `both_arms` 规划组后，选择 `park` 或设置目标，点击 Plan 检查轨迹，再 Execute 控制实物。

此入口不启动 Gazebo。柜体高 0.60 m，支撑底部离地 0.28 m；柜宽 1.20 m、深 0.40 m、柜体近侧 X=0.50 m（已根据指尖参考校准，宽深仍为占位尺寸），执行前按现场实测修改，避免规划场景与实际障碍物不一致：

```zsh
bash /home/a/Nero-7/scripts/start_dual_nero_moveit.sh \
  support_height:=0.28 cabinet_height:=0.60 \
  cabinet_width:=1.20 cabinet_depth:=0.40 cabinet_front:=0.50
```

只加载反馈、不自动使能可追加原有 `auto_enable:=false`（不等于失能已使能的机械臂）。配置完成后，柜体应出现在 RViz Planning Scene 中；若场景节点报错或柜体未显示，应先修复场景再规划执行。

已通过不接 CAN 的测试：7 项模型/路由测试；左右臂与双臂规划及 mock 执行；空闲启动不发送指令；柜体/地面/支撑已进入规划场景；park 无碰撞。报告见 `hardware_ws/validation/dual_moveit/cabinet_scene_report.json` 和 `mock_report.json`。本次系统无 CAN 接口，没有运行实物驱动或验证实物运动。

## Gazebo 对照预览

2026-10-27：固定双臂上身模型，复用现有 Gazebo Classic + MoveIt 控制，不连接 CAN。

```zsh
bash /home/a/Nero-7/scripts/start_cabinet_sim.sh
```

先关闭之前的同端口仿真。默认 ROS_DOMAIN_ID=76，Gazebo master 端口 11476。
柜体场景默认以双臂 `park` 启动：每臂 `[0,85,0,5,0,0,0]°`。RViz 中 `left_arm`、`right_arm`、`both_arms` 均可选择名为 `park` 的预设。需要原来的零位启动时追加 `startup_arm_pose:=home`。只设置仿真初始关节状态，不调用实机 `park.sh`。
已核对运行时 URDF 的初始值与 park 一致；物理启动后控制器接管存在少量偏差，本次左右臂反馈最大偏差分别约 0.92°、0.86°，初始姿态碰撞检查通过。具体值见 `report.json`，不将初始设定值等同于零误差实测反馈。
需要覆盖隔离设置时使用 `NERO_CABINET_DOMAIN_ID` 和 `NERO_CABINET_MASTER_URI` 环境变量，避免继承实机终端的 ROS 域。

## 几何与测量基准

- `support_height=0.28`：地面到中间支撑底部，即模型 `base_link` 原点的高度。用户确认测的是支撑底部，当前数值约 28 cm，仍待精确测量。
- 模型固定关节 `left_arm_joint/right_arm_joint` 的安装位置为 `(-0.002, ±0.1, 0.59)` m。因此双臂安装基座原点离地为 `0.28 + 0.59 = 0.87 m`。这依赖现有模型与实物中间支撑尺寸一致，并非实测的法兰高度。
- 柜顶离地 `cabinet_height=0.60` m，柜体底部落地，实体碰撞盒包含柜顶。
- 柜宽 `cabinet_width=1.20` m，柜深 `cabinet_depth=0.40` m，均为待测占位值。
- 前方定义为模型 +X。柜子靠机器人一面的 X 坐标 `cabinet_front=0.50` m，相对中间支撑原点，不是相对底座最前缘的间隙。柜中心为 `(0.70, 0, 0.30)` m。
- 下方灰色 `0.40 × 0.35 × 0.28 m` 底座仅为支撑占位体；未建移动底盘动力学。

Gazebo 与 MoveIt 使用同一份柜体和支撑尺寸生成碰撞物。世界地面 z=0；机器人不是通过 spawn 偏移单独抬高，而是修改同一 URDF 的 world→base_link 固定变换，因此 TF、Gazebo 与 MoveIt 的高度一致。原有不带参数的 gazebo_moveit.launch.py 仍保持原先的落地模型和无柜体场景。

## 调整尺寸

```zsh
bash /home/a/Nero-7/scripts/start_cabinet_sim.sh \
  support_height:=0.28 cabinet_height:=0.60 \
  cabinet_width:=1.20 cabinet_depth:=0.40 cabinet_front:=0.50
```

只开 Gazebo：追加 `use_rviz:=false`；后台物理验证：追加 `gui:=false use_rviz:=false`。

最直接的测量是地面到左右臂安装基座原点/法兰中心的高度；若只量中间支撑底部，需要再确认底部到安装中心的垂直距离是否真为模型中的 59 cm。安装高度变化可直接改 `support_height`，无需改机械臂连杆尺寸。

## 构建

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select dual_nero_moveit_config --symlink-install
```

场景用于仿真几何与规划验证；模型已有相机和支架外形，并不代表实物当前载荷或标定已核实。

## 本次验证

已构建并启动 Gazebo/RViz。`sim_ws/validation/cabinet/report.json` 记录了柜体与支撑的规划碰撞几何、world→base_link=0.28 m、左右臂安装原点 z=0.87 m、五个控制器 active、初始全机器人状态无碰撞。Gazebo 独立查询得到柜中心 `(0.85, 0, 0.30)` m，模型链接信息中 base_link 的 z=0.28 m；见同目录 `gazebo_model.txt` 与 `gazebo_geometry.json`。

该目录还保存了生成的 `cabinet.world`，可查看完整场景几何。正常启动时仍按命令参数重新生成世界。

重跑只读验证：

```zsh
source /opt/ros/humble/setup.zsh
ROS_DOMAIN_ID=76 python3 /home/a/Nero-7/sim_ws/validation/cabinet/verify_scene.py
```

边界：未执行运动轨迹或实机控制。第一次验证进程退出时 MoveIt/RViz 出现 -11；重新启动后的场景、控制器与碰撞检查通过，退出稳定性未解决。

## RViz 启动后柜子未显示的修复

2026-10-27：已确认 MoveIt 服务中存在柜体，但场景节点先于 RViz 订阅退出；补发完整 PlanningScene 后用户确认柜子显示。场景节点现在等待查看器订阅建立，再从 MoveIt 获取并发送完整场景。未改 CAN 驱动和轨迹控制，也没有重启当前实物进程。独立 domain 94 的延迟订阅回归测试通过。

## 2026-10-27 柜前沿位置校准

用户确认：当前右臂指尖水平越过实际柜前沿约 3.5 cm。读取真实关节反馈，world→base_link 高 .28 m；TCP 本体坐标约 (.35995,-.40004,.41994)，world Z≈.69994 m。结合当前 URDF 两侧手指碰撞 STL 的变换，最前端 world X≈.53496 m，因此 cabinet_front≈.53496-.035=.49996，取 .50 m。柜体从原前沿 .65 m 向机器人平移 .15 m；宽1.2、深.4、高.6不变。模型指尖尺寸与用户尺量均有误差，此值是现场参考校准，不是高精度外参标定。

硬件 RViz 场景和 Gazebo 柜体入口默认值均改为 .50；当前 MoveIt 场景只更新 front_cabinet，原驱动与关节状态不受修改。手指当前最低 Z≈.672 m，在柜顶 .60 m 之上，所以水平越过前沿不等于模型穿入实体柜体。读数报告：`/home/a/Nero-7/hardware_ws/validation/dual_moveit/cabinet_reference.json`。前文 Gazebo 验证中 .85 m 的柜中心是校准前的历史结果。

# AIR + 双 NERO 整车仿真

组合包复用已有 AIR 与双 NERO 模型。默认将双臂支架安装在 AIR 坐标系 `(0, 0, 0.275)` m，安装姿态为零；这是仿真安装方案，尚未按实车安装尺寸校准。

使用一套 robot_description、controller_manager 与 joint_state_broadcaster。移除双臂原有 world 固定关节，保留左右机械臂和夹爪的原有关节名，将双臂原 base_link 重命名为 dual_nero_base_link；相机冲突名称也做映射。模型树为 `odom → base_link(AIR) → arm_mount_plate → dual_nero_base_link → 左右机械臂`。

## 运行（zsh）

整车默认从 `relaxed` 双臂下垂姿态生成（左右 joint2 均为 1.45 rad，其余臂关节为 0）；这是仿真模型初始状态，启动后无需先执行收臂轨迹。原有 `home` / `ready` 命名目标保留原定义，新增 `relaxed` 可供左右臂及 both_arms 规划使用。参数 `startup_arm_pose:=home` 可恢复旧零位启动；便利店场景请使用默认下垂姿态，零位横展的手臂可能与邻近货架相交。

整车运行时只需启动下面这一套，不要同时在同一 ROS 域内启动独立双臂或独立底盘。

终端 1：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=0
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
ros2 launch mobile_dual_nero gazebo_moveit.launch.py
```

终端 2 控制底盘：

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 run ranger_air_gazebo air_keyboard --ros-args -p speed:=0.1 -p turn:=0.2
```

W/S 前后、A/D 左右横移、Q/E 原地旋转、空格停车。按运动键后持续运动，空格停止。Ctrl-C 退出键盘；终端 1 Ctrl-C 结束整套仿真。之前独立小车使用过域 37，整车默认使用域 0，因此键盘终端也必须切换到域 0。

底盘停车后，在 RViz 的 MotionPlanning 中选择 left_arm、right_arm、both_arms、left_gripper 或 right_gripper 规划组，选择 home/ready 或 open/close，然后 Plan / Execute。也可以拖动单臂末端交互标记进行位姿规划。当前验证流程为停车操作双臂；尚未实现底盘与双臂的同步全身规划或自动抓取。

安装参数可覆盖（单位 m/rad）：

```zsh
ros2 launch mobile_dual_nero gazebo_moveit.launch.py \
  mount_xyz:="0 0 0.275" mount_rpy:="0 0 0"
```

`mount_xyz` 表示双臂支架底部在 AIR base_link 中的位置，适配板厚 12 mm，位于安装面下方。调整安装位置后，应重新做碰撞、稳定性和规划验证。

## 实现与边界

- 保留原独立模型和启动入口；组合模型由 Python 构建，不复制一份容易分叉的大型 URDF。
- 整车 26 个控制关节：底盘 8、双臂 14、夹爪 4；一个 gazebo_ros2_control 硬件系统和一个 controller_manager。
- 七个控制器：joint_state_broadcaster、steering_controller、wheel_controller、左右臂和左右夹爪轨迹控制器。
- `/cmd_vel` 控制底盘；原有 FollowJointTrajectory 控制双臂和夹爪。
- `/joint_states` 来自实际 Gazebo 关节反馈。轮式里程计发布 odom → base_link，robot_state_publisher 发布安装链与机械臂 TF。所有节点使用仿真时间。
- MoveIt 通过 odom → base_link 的平面虚拟关节接收底盘位姿，底盘轮组标为 passive，不加入机械臂规划组。底盘驶离起点后仍可规划机械臂。
- MoveIt 地面固定在 odom 中，地面接触仅豁免轮胎；底盘、支架、机械臂和夹爪仍参与地面碰撞检查。原双臂自碰撞豁免表经过名称映射复用。
- RViz 固定坐标系为 odom，相机跟随 base_link，视距覆盖整车。
- Gazebo 启动会将 AIR、双臂及组合包的 share 目录加入客户端模型搜索路径。isolated colcon 安装中，这些目录分别位于不同包前缀；漏加会出现 RViz 正常而 Gazebo 只有适配板方块，并在客户端日志中报 model:// 网格找不到。
- 总模型质量约 32.35 kg（包含 0.6 kg 适配板）。这只是源模型质量之和，未验证 AIR 实车承载能力或惯量标定。
- 整车默认平移限速 0.2 m/s、角速度限速 0.4 rad/s；加速度分别为 0.15 m/s²、0.3 rad/s²；转向限速 0.8 rad/s；保留指令和关节反馈超时停止。
- 底盘采用简化接触碰撞，忽略部分传感器与转向支架碰撞；驱动与机械臂为 GazeboSystem 的理想速度/位置接口。尚未验证大幅伸臂时的抗倾覆边界、载物、坡道、传感器输出或真实硬件。
- 当前 MoveIt 版本在 SIGINT 关闭时可出现析构段错误；运行期间的规划执行已验证，关闭阶段异常未在本包内修复。

## 验证

记录目录：`/home/a/Nero-7/sim_ws/validation/mobile_dual_nero/`。

- `run_01/assembly.json`：Gazebo 中底盘和两侧腕部一起平移约 0.467 m，刚性安装关系通过。
- `run_01/arms.json`：装车后左右臂、双臂 ready/home、两侧夹爪开合及两侧末端 IK 规划，共 12 项检查通过。
- `run_01/motion/`：保留首次发现固定时间步长问题的失败记录。
- `run_02/motion/report.json`：修正控制定时后，带双臂底盘 8 项运动与停车测试通过，含独立 Gazebo 真值和轮式里程计比较；`telemetry.csv` 保存完整测量。
- `run_02/arms.json`：整车平移、旋转并停下后再次执行机械臂验证的 12 项通过记录。运行控制台日志不纳入版本管理。
- `colcon test` 覆盖模型树、唯一关节/控制系统、SRDF 引用、安装参数、底盘运动学和控制定时降频回归。

重新验证前先停车并关闭键盘，确认没有其他 `/cmd_vel` 发布者。验证命令在整车同一 ROS 域中执行：

```zsh
ros2 run mobile_dual_nero verify_assembly \
  --output /home/a/Nero-7/sim_ws/validation/mobile_dual_nero/new_run/assembly.json \
  --ros-args -p use_sim_time:=true

ros2 run ranger_air_gazebo verify_motion --entity mobile_dual_nero \
  --output-dir /home/a/Nero-7/sim_ws/validation/mobile_dual_nero/new_run/motion \
  --ros-args -p use_sim_time:=true

ros2 launch mobile_dual_nero verify_arms.launch.py
```

如果修改了安装参数，`verify_arms.launch.py` 也须传入相同 mount_xyz/mount_rpy。

## 货物整理环境

默认启动 `warehouse` 环境，含两组四层货架、两张工作台、一瓶动态水瓶和六个静态示例货箱。使用 `environment:=empty` 恢复原平地环境。尺寸及位置见 `mobile_dual_nero/environment.py`，Gazebo 和 MoveIt 使用同一几何定义。水瓶位姿从 `/gazebo/model_states` 同步到 MoveIt；初始 world 与 odom 对齐，底盘轮式里程计漂移尚未校正。抓取、搬运及导航尚未验证。

## 便利店场景（MID360 布置测试的环境底图）

```zsh
cd /home/a/Nero-7/sim_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select mobile_dual_nero --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.zsh
ros2 launch mobile_dual_nero gazebo_moveit.launch.py environment:=convenience_store
```

平面范围为 12 × 10 m，原点是机器人初始位置；x 正向为东，y 正向为北。入口在南侧 `(0, -5)`，宽 2 m。三排双面货架中心在 `(-2.05, 2.3)`、`(0, 2.3)`、`(2.05, 2.3)`，单排宽 0.85 m、长 3 m，排间净宽 1.20 m（包含价签和商品的外轮廓检查）；层板上表面高 0.20 / 0.60 / 1.00 / 1.45 m，货架立柱和分类牌顶端为 1.80 m，商品不超高。西侧靠墙商品架也为 1.80 m 高，东侧为三个 2.15 m 高冷柜；西南是收银台和咖啡台，东南是低促销岛，后排有不对称纸箱。原点周围半径 0.70 m 内没有环境碰撞物。

参数和商品布局在 `mobile_dual_nero/convenience_store.py`，与旧仓库场景独立。货架、商品、柜体、收银台和墙体的几何同时生成 Gazebo 碰撞和 MoveIt 障碍物；地砖缝、价签、商品标签和冷柜门上的饮料图案仅作装饰，不进入 MoveIt。冷柜门为不透明几何近似，没有模拟玻璃透射、多径或反射率；场景现包含距地 3.10 m 的顶板碰撞（仅隐藏顶板视觉，便于观察），南侧上半部分也参与碰撞；半透明显示仅是预览效果，没有玻璃光学模型。通道宽度是场景尺寸，不代表伸臂行驶已经通过碰撞验证。

此入口仍使用现有机器人模型，尚未安装 MID360、生成雷达点云或接入 SLAM。后续须接入带真实遮挡的雷达仿真与 TF/IMU，再对比同一路径上的收臂、伸臂和运动工况；仅凭当前场景无法判断安装位置或建图干扰。切回旧整理任务使用 `environment:=warehouse`，旧任务目标坐标不适用于便利店。


便利店验证记录见 `sim_ws/validation/convenience_store/acceptance.json`，复核脚本为同目录 `verify_scene.py`（只读，不发布运动命令）。2026-10-21 初版（未装雷达、未加顶板）在独立 ROS 域 89、Gazebo master 11489 完成场景加载与下垂姿态验证：467 个环境模型，其中 214 个物理障碍物全部进入 MoveIt，另有 1 个 MoveIt 地面；整车当前状态碰撞检查通过。10 秒静置采样中，底盘最大倾角约 0.15°、距生成原点最大约 5.3 mm。肩关节与目标相差约 0.006 rad；右腕 joint7 的反馈相对初值仍偏约 0.146 rad，本轮未修改已有位置接口/关节动力学，因此该记录仅验证下垂外形、静置稳定性和当前碰撞状态，不表示全关节高精度保持。9 项包内测试通过。未运行机器人行驶、抓取、MID360 点云或 SLAM 验证。

## 装 MID360 并运行 Super-LIO

本机已接入可选的 `nero_mid360_sim` 插件和 `store_superlio.launch.py`。推荐从项目根目录运行：

```zsh
cd /home/a/Nero-7
bash scripts/run_store_mapping.sh mid360_mount:=chassis_up
# 再次执行会自动重启本工程仿真，切换胸前倒装：
bash scripts/run_store_mapping.sh mid360_mount:=chest_down
```

两个预设及坐标定义见 `config/mid360_mounts.yaml`，可追加 `mid360_xyz:="0.18 0 0.78" mid360_rpy:="3.141592653589793 0 0"` 覆盖。此时单位为米、弧度，父坐标系是 AIR `base_link`；胸前位置不会随手臂关节运动。新脚本将完整仿真放在 ROS 域 89 / Gazebo 11489；键盘或机械臂控制终端须使用相同域。默认启动姿态仍为双臂下垂。

详细流程、几何/IMU 近似、算法近距过滤与证据边界见 `../nero_mid360_sim/README.md`。普通 `gazebo_moveit.launch.py` 仍默认不装雷达；需要用它单独生成雷达时传 `mid360_mount:=chassis_up|chest_down`。

建图对照实验中，`air_keyboard` 还支持 **T** 切换双臂小幅摆动（默认关闭，左右臂交替前后摆动，±17.2°、8 秒一轮）；空格同时停车和停止摆臂。用法和前置条件见 [建图说明](../nero_mid360_sim/README.md)。


整车上身现带 0～0.80 m 竖直升降，`lift_height:=0.0` 控制启动升降量。键盘 R/F 每次升降 5 cm，空格停止全部；`ros2 run ranger_air_gazebo set_lift_height --height 0.40` 可设绝对升降量。胸前 MID360 随上身升降，底盘 MID360 不动。详见 [上身升降与雷达随动](../nero_mid360_sim/README.md)。

MID360 还支持头顶预设 `mid360_mount:=head_tilt mid360_tilt_deg:=45`，原点在上身最低时约 z=1.02 m，随上身升降。倾角单位度，可设置 -90～90，正值朝机器人前方倾斜；原有底盘/胸前安装不受此参数影响。默认 45°，改角度后重启生效。

默认收臂姿态已将左右 J2 从 1.25 rad 收到 **1.45 rad（约 83.1°）**。其余臂关节初始值仍为 0。按碰撞模型（含夹爪与腕部相机）计算，横向包络从约 1.09 m 降至 0.83 m；在 1.20 m 净通道中央时名义单侧余量约 0.18 m。这是初始姿态几何余量，不代表整段通道行驶已通过验收。键盘 T 摆动的 J2 启动检查已同步支持新姿态；更改默认姿态需要重新生成/启动仿真模型。

整车初始位姿现在按 URDF 四个零转向轮胎的接触平面计算，约留 3 mm 离地间隙，避免原先水平悬空 3 cm 后落地再调姿。保留 CAD 前后轮约 1 mm 的轴心高度差，因此底盘静态约有 -0.15° 俯仰，这不等于持续振荡。轮胎法向接触 kd 改为 3000；升降导轨的质心及均匀盒子惯量已校正。启动与短程运动证据见 `sim_ws/validation/base_rocking/README.md`；未锁定底盘自由度。

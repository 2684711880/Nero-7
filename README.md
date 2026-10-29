# NERO-7：双臂控制、视觉戳洞与 RANGER AIR 仿真

ROS 2 Humble / Gazebo Classic 11 整车仿真：RANGER AIR 四轮独立转向底盘搭载双 NERO 机械臂，支持底盘键盘控制及 MoveIt 2 机械臂规划执行。

## 视觉戳洞 demo

左臂侧夹 40 × 40 cm 方板，右腕 D405 深度识别约 9 cm 圆孔；右臂先到观察位，必要时转腕，再预览并确认执行孔前接近和前送。

**[demo README：环境、入口、数据与验证范围](demos/hole/README.md)** · [实机完整命令](docs/real_hole_workflow.md) · [Gazebo 仿真命令](docs/sim_hole_approach.md)。实机每段先 RViz 预览，输入 `do` 才执行；左臂保持反馈只读。

检测代码已随仓库保存，无需本机 ArmHub 目录。左右相机标定分别保留在 `config/handeye/left/`、`config/handeye/right/`；临时采集和轨迹计划不上传。实机工具尺寸仍为现场暂定值，精度与碰撞模型限制见 demo 文档。

## 已验证

- 26 个控制关节、7 个控制器，由同一 robot_description、controller_manager 和 joint_state_broadcaster 管理。
- 底盘前后移动、左右横移、双向旋转、组合运动和命令中断停车：8 项测试通过。
- 双臂随底盘刚性移动；底盘移动、旋转并停下后，双臂和夹爪的 12 项规划/执行检查通过。
- MoveIt 地面碰撞检查，轮式里程计及完整 TF；RViz 相机跟随底盘。
- Gazebo 网格搜索路径回归检查：41 个独立网格全部可解析。
- RViz 与原话题仿真接收端的机械臂速度/加速度缩放为 0.8；三路任务的独立机械臂适配器使用 0.3。
- 三路动作任务：底盘、左臂、右臂独立顺序执行，通过依赖协调；任务执行包 50 项单元/回归测试通过。
- 当前水瓶任务完成侧向抓取、直接下降放置与返回原点；左臂仅调整 J2 到 +60°。
- C++ 底盘执行器支持纯平移、纯转向与同步移动转向；分段和组合夹水任务均完成 Gazebo 验收，取消、超时与反馈失效检查通过。
- 组合运动平顺性修复：同一任务对照中，搬运/回程中段停轮占比从约 15.5% / 36.1% 降至 0% / 0%；当前底盘与任务执行包合计 59 项单元测试通过。证据见 [诊断记录与速度曲线](sim_ws/validation/base_smoothness/README.md)。

当前阶段已完成预设仓库场景下的基础移动抓放闭环，保留原分段路线与边移动边转向两种任务入口。本阶段发布前构建及检查记录见 [publication_base_motion_20261015.json](sim_ws/validation/publication_base_motion_20261015.json)。

这些是简化模型的平地仿真结果，不是实车承载、抗倾覆、电机动力学或同步全身规划验证。默认双臂安装面为 AIR base_link 中 `(0, 0, 0.275)` m，可通过启动参数调整。

## 环境与编译

已测试 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11.10.2。需要安装 ROS 2 开发环境、Gazebo ROS 集成、gazebo_ros2_control、ros2_control/ros2_controllers、MoveIt 2、KDL、OMPL、xacro、RViz2、colcon 和 rosdep。

实机相关代码、标定样本与设备序列号随仓库保存；这些标定仅适用于记录的设备和安装状态，不能直接推广到另一台机器人。构建产物、虚拟环境、相机采集会话和下载的第三方工作树不随仓库上传。

实机环境可用 `bash scripts/build_official_control.sh` 准备，脚本会应用 `deps/agx_arm_ros-local.patch`。重力补偿依赖可用 `bash scripts/build_nero_gravity.sh` 准备；手眼插件使用 `bash scripts/build_handeye.sh`。这些准备脚本不会替代实际硬件验收。336L 的 Python SDK 目前仍需单独配置，可通过 `ORBBEC_HANDEYE_PYTHON` 指向包含 `pyorbbecsdk` 和 ROS Python 依赖的解释器；现有默认路径是本机环境。

克隆本仓库并编译整车：

```bash
git clone https://github.com/2684711880/Nero-7.git
cd Nero-7
bash scripts/build.sh
```

默认编译整车和三路任务执行器需要的本地包。厂商 `agx_arm_sim` 作为固定版本子模块保留；整车已包含所需模型，不依赖下载全部厂商资源。需要独立厂商示例时运行：

```bash
bash scripts/bootstrap_vendor.sh
```

该脚本递归初始化固定版本子模块，并幂等应用 `deps/agx_arm_sim-cmake.patch`（可选 MuJoCo 资源缺失时仍允许编译）。不会将本地修改推送到厂商仓库。

## 启动整车

以下命令适用于 zsh；bash 用户将 setup.zsh 换成 setup.bash。请在仓库根目录执行。

终端 1：

```zsh
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
ros2 launch mobile_dual_nero gazebo_moveit.launch.py
```

终端 2：

```zsh
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 run ranger_air_gazebo air_keyboard --ros-args -p speed:=0.1 -p turn:=0.2
```

W/S 前后、A/D 横移、Q/E 旋转、空格停车。运动按键会保持指令；按空格停止，Ctrl-C 退出键盘。底盘停车后，在 RViz MotionPlanning 中选择左右臂、双臂或夹爪规划组，使用 Plan / Execute。终端 1 Ctrl-C 退出整套仿真。

减少图形负载，可只保留一个界面：

```zsh
# Gazebo 后台仿真 + MoveIt RViz
ros2 launch mobile_dual_nero gazebo_moveit.launch.py gui:=false
# Gazebo 图形界面，不打开 RViz
ros2 launch mobile_dual_nero gazebo_moveit.launch.py use_rviz:=false
```

## 货物整理环境

整车启动默认加载 `warehouse`：两组四层货架、两张 1.2 × 0.7 m 工作台（台面高 0.6 m）、一瓶 0.52 kg 的简化水瓶，以及六个静态示例货箱。机器人从中央启动，桌间净宽 2.8 m；水瓶在右侧取物台，左侧桌面留空。

```zsh
ros2 launch mobile_dual_nero gazebo_moveit.launch.py
# 需要原来的平地场景时：
ros2 launch mobile_dual_nero gazebo_moveit.launch.py environment:=empty
```

修改该文件顶部的 `TABLE_HEIGHT = 0.6` 可统一调整两张桌子的桌面高度（米）；桌板中心、桌腿长度和水瓶初始高度随之计算。修改后重新构建 `mobile_dual_nero` 并重启仿真。

几何尺寸和位置统一定义在 `sim_ws/src/mobile_dual_nero/mobile_dual_nero/environment.py`，启动时同时生成 Gazebo 世界和 MoveIt 碰撞体。货架采用分层碰撞几何，层间留空；水瓶为可运动刚体，桌子、货架和示例货箱为固定模型。`sync_bottle` 将水瓶仿真位姿以最高 5 Hz 更新到 MoveIt，移除或附着后不会重新创建世界物体。通过 `sim_grasp:=true` 可启用经过几何检查的 Gazebo 固定关节抓取约束；自动搬运任务见下方三路动作任务。该约束不是摩擦夹持或实机力控制模型。

环境的 Gazebo world 与初始 odom 对齐。底盘移动后的轮式里程计误差仍会导致规划场景与真值偏差；尚未接入定位校正或导航。货架最高层仅用于布局展示，可达性需要后续结合底盘停靠和机械臂规划验证。

运行验证记录见 `sim_ws/validation/warehouse/report.json`；布局截图见 `sim_ws/validation/warehouse/scene.png`。

## 包与记录

| 路径 | 内容 |
| --- | --- |
| `sim_ws/src/ranger_air_description` | 官方 AIR 模型与独立预览 |
| `sim_ws/src/ranger_air_gazebo` | 底盘控制、里程计、键盘和运动验证 |
| `sim_ws/src/dual_nero_moveit_config` | 独立双臂模型、控制与 MoveIt 配置 |
| `sim_ws/src/mobile_dual_nero` | 整车组合、安装参数、MoveIt 与验证 |
| `sim_ws/src/agx_arm_sim` | 固定版本厂商子模块，保留独立示例 |
| `sim_ws/src/nero_task_interfaces` | 带目标标识、结果与取消的机械臂 Action |
| `sim_ws/src/nero_task_executor` | Python 三路调度、C++ 机械臂适配器、任务 JSON 与注释版 |
| `hardware_ws/src/nero_official_commander_cpp` | 官方风格 C++ 运动调用接口与示例 |
| `sim_ws/validation` | 验证报告、模型快照和测量数据，含保留的失败记录 |

详细说明见 [整车 README](sim_ws/src/mobile_dual_nero/README.md)、[底盘控制说明](sim_ws/src/ranger_air_gazebo/CONTROL.md)。验证汇总见 [summary.json](sim_ws/validation/mobile_dual_nero/summary.json)。编译产物、运行日志和 Python 缓存不纳入版本管理。

```bash
source /opt/ros/humble/setup.bash
source sim_ws/install/setup.bash
cd sim_ws
colcon test --packages-select ranger_air_gazebo mobile_dual_nero
colcon test-result --verbose
```

ROS 节点运行时目录不同不会影响模型资源；Gazebo 启动已加入不同 colcon 包前缀的资源搜索目录。

## 三路动作任务与水瓶搬运

上层发布一份任务 JSON，底盘、左臂、右臂三条序列分别顺序执行，跨序列通过 `wait_for` 等待成功结果。`action_id` 是唯一标签，执行顺序由数组排列和依赖决定，不按编号排序。

当前 [任务数据](sim_ws/src/nero_task_executor/examples/bottle_transfer.json) 为 `bottle_transfer_004`；[逐项注释版](sim_ws/src/nero_task_executor/examples/bottle_transfer.jsonc) 供阅读，执行仍使用标准 JSON。流程是三路准备 → 右臂抓取抬起 → 底盘移动转向左桌 → 右臂直接下降松开 → 底盘返回原点并恢复初始朝向。左臂只给 J2 设置 +60° 目标，其余关节目标保持动作开始时的反馈值。

以下三个终端均从仓库根目录执行。先结束旧仿真的完整 launch，再启动新场景；不要只关闭 Gazebo GUI。

终端 1：

```zsh
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=75
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
ros2 launch mobile_dual_nero gazebo_moveit.launch.py \
  command_bridge:=false sim_grasp:=true ground_truth_odometry:=true \
  gui:=true use_rviz:=false
```

模型、控制器与 MoveIt 就绪后，终端 2：

```zsh
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=75
ros2 launch nero_task_executor simulation.launch.py \
  state_file:=/tmp/nero_task_$(date +%s).json
```

左右臂适配器与执行节点就绪后，终端 3：

```zsh
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=75
ros2 run nero_task_executor submit_task \
  sim_ws/src/nero_task_executor/examples/bottle_transfer.json
```

重新演示前停止三个终端并重建场景。任务中断后不要直接换状态文件重发，先确认停止并重置仿真。`task_executor` 单独启动默认使用 mock，不发送运动；实际仿真必须使用上述 simulation launch。

已验证当前场景的一次完整抓放返回，水瓶水平误差约 2.9 mm、底盘返回误差约 5.3 mm；另验证两次连续原地转向。实际低速收尾停滞的失败证据与修复记录见 [验收报告](sim_ws/validation/task_executor/j2_turn_fix/README.md)。这些数值不代表任意初始状态或实机精度。

实现入口、状态机、取消与恢复规则见 [任务执行器说明](sim_ws/src/nero_task_executor/README.md) 和 [阅读入口](docs/three_lane_tasks.md)。独立规划不提供全身动态避碰，底盘路线没有障碍规划，当前未接入实机 backend。

## 官方机械臂控制代码

当前 C++ 入门程序是最小直线运动，见 [最小直线程序](docs/cpp_straight_line_first.md)。
历史画圆示例讲解见 [C++画圆入门](docs/cpp_xy_circle_first.md)。

当前C++入口按官方SDK风格调用 `move_j/move_p/move_l/move_c`，跨语言链路与运行说明见
[官方风格C++接口](docs/official_sdk_cpp_api.md)。

官方 ROS 驱动与 Python SDK 已作为固定版本子模块放在 `third_party/`。
独立 `hardware_ws` 构建官方控制/消息包及 C++ 调用层，默认只检查输入，不发送运动命令。
说明见 [官方控制工作空间](hardware_ws/README.md) 和 [源码核查](docs/official_arm_control_audit.md)。

```bash
bash scripts/build_official_control.sh
source scripts/official_control_env.sh
```

此工作空间与上述整车仿真分别使用。实机视觉孔任务已增加独立流程，当前验证范围和操作命令见 [实机说明](docs/real_hole_workflow.md)。

用同一个C++发布程序控制Gazebo的步骤见 [C++话题控制仿真](docs/cpp_topic_simulation.md)。
整车启动默认带有直线/圆弧命令接收端。
当前初学入口已精简成仅从A到B画直线，先看 [最小直线程序与坐标系](docs/cpp_straight_line_first.md)。

双桌搬瓶的 C++ 任务与运行步骤见 [搬瓶测试](docs/cpp_bottle_pick_place.md)。

## 来源与限制

第三方模型来源与许可证见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。上游资源保留原有许可证声明，不宣称这些模型为本项目原创。

部分内层说明及历史验证文件记录原开发机绝对路径 `/home/a/Nero-7`；迁移机器时使用本 README 的相对路径命令，并为验证工具传入自己的输出目录。当前 MoveIt 安装版本在关闭阶段可能出现析构段错误；这与已经通过的运行期间规划/执行验证区分记录。

本次发布前的干净克隆构建与 35 项测试记录见 [publication_20261015.json](sim_ws/validation/publication_20261015.json)。

可选底盘纯平移、纯转向及同步组合运动已接入独立 C++ 跟踪器，原分段模式保留。说明与示例见 [base_motion_modes.md](docs/base_motion_modes.md)。

## MID360 安装方案与 Super-LIO 便利店建图

```zsh
cd /home/a/Nero-7
bash scripts/run_store_mapping.sh mid360_mount:=chassis_up
# 直接重新运行，自动切换胸前倒装：
bash scripts/run_store_mapping.sh mid360_mount:=chest_down
```

传 `mid360_xyz:="0.18 0 0.78" mid360_rpy:="3.141592653589793 0 0"` 可覆盖安装位姿。安装预设文件为 `sim_ws/src/mobile_dual_nero/config/mid360_mounts.yaml`（AIR base_link 下的 m/rad）。此功能使用 Gazebo 碰撞几何射线和理想 IMU，不启动实体雷达驱动。完整说明与编译命令见 [MID360 仿真说明](sim_ws/src/nero_mid360_sim/README.md)，短程验收和自体回波测试数据在 `sim_ws/validation/mid360_superlio/`。

建图后端可切换：`algorithm:=fastlio2` 或 `algorithm:=superlio`，统一用 `lio_blind:=0.3`（单位米）修改算法近距剔除。不传阈值时，FAST-LIO2 为 0.3 m、Super-LIO 为 2.0 m。RViz 中勾选 `Raw MID360 (before LIO filter)` 可检查过滤前的原始回波。

建图对照实验中，`air_keyboard` 还支持 **T** 切换双臂小幅摆动（默认关闭，左右臂交替前后摆动，±17.2°、8 秒一轮）；空格同时停车和停止摆臂。用法和前置条件见 [建图说明](sim_ws/src/nero_mid360_sim/README.md)。
## 手动 ChArUco 手眼标定

与之前相同的 RViz 手动采样界面：`bash scripts/run_handeye.sh --can can1`。
在 Calibrate 页 Take Sample → Calibrate，并主动保存样本与结果。

末端 RealSense、手动摆臂拍照的眼在手上流程见 [操作说明](docs/handeye_charuco.md)。
用户独立启动重力补偿，采集程序被动读取指定 CAN 的七轴反馈。
入口为 `bash scripts/handeye.sh collect --can <接口> --marker-mm <实测编码黑框边长>`；
方格 40 mm，板为按 ID 排列的 7×5 格。支持原图/位姿保存、离线求解和新姿态独立验证。

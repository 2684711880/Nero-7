# RANGER AIR + 双 NERO Gazebo 仿真

ROS 2 Humble / Gazebo Classic 11 整车仿真：RANGER AIR 四轮独立转向底盘搭载双 NERO 机械臂，支持底盘键盘控制及 MoveIt 2 机械臂规划执行。

## 已验证

- 26 个控制关节、7 个控制器，由同一 robot_description、controller_manager 和 joint_state_broadcaster 管理。
- 底盘前后移动、左右横移、双向旋转、组合运动和命令中断停车：8 项测试通过。
- 双臂随底盘刚性移动；底盘移动、旋转并停下后，双臂和夹爪的 12 项规划/执行检查通过。
- MoveIt 地面碰撞检查，轮式里程计及完整 TF；RViz 相机跟随底盘。
- Gazebo 网格搜索路径回归检查：41 个独立网格全部可解析。
- 机械臂默认速度/加速度缩放为 0.8（关节上限的 80%）。

这些是简化模型的平地仿真结果，不是实车承载、抗倾覆、电机动力学或同步全身规划验证。默认双臂安装面为 AIR base_link 中 `(0, 0, 0.275)` m，可通过启动参数调整。

## 环境与编译

已测试 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11.10.2。需要安装 ROS 2 开发环境、Gazebo ROS 集成、gazebo_ros2_control、ros2_control/ros2_controllers、MoveIt 2、KDL、OMPL、xacro、RViz2、colcon 和 rosdep。

克隆本仓库并编译整车：

```bash
git clone https://github.com/2684711880/Nero-7.git
cd Nero-7
bash scripts/build.sh
```

默认只编译整车需要的本地包。厂商 `agx_arm_sim` 作为固定版本子模块保留；整车已包含所需模型，不依赖下载全部厂商资源。需要独立厂商示例时运行：

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

## 包与记录

| 路径 | 内容 |
| --- | --- |
| `sim_ws/src/ranger_air_description` | 官方 AIR 模型与独立预览 |
| `sim_ws/src/ranger_air_gazebo` | 底盘控制、里程计、键盘和运动验证 |
| `sim_ws/src/dual_nero_moveit_config` | 独立双臂模型、控制与 MoveIt 配置 |
| `sim_ws/src/mobile_dual_nero` | 整车组合、安装参数、MoveIt 与验证 |
| `sim_ws/src/agx_arm_sim` | 固定版本厂商子模块，保留独立示例 |
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

## 来源与限制

第三方模型来源与许可证见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。上游资源保留原有许可证声明，不宣称这些模型为本项目原创。

部分内层说明及历史验证文件记录原开发机绝对路径 `/home/a/Nero-7`；迁移机器时使用本 README 的相对路径命令，并为验证工具传入自己的输出目录。当前 MoveIt 安装版本在关闭阶段可能出现析构段错误；这与已经通过的运行期间规划/执行验证区分记录。

# 一个文件写整个机械臂任务

编辑 `/home/a/Nero-7/hardware_ws/src/nero_official_commander_cpp/src/robot_task.cpp`，在 `task(robot)` 中按顺序编写全部流程。当前示例：点到点到达夹瓶准备位置和朝向 → 张开夹爪 → 沿机器人 +X 直线接近瓶子 → 闭合 → 直线上抬 5 cm → 一条直线同时 X+10 cm、Y+20 cm → 直线下降 4 cm → 打开夹爪。

## 当前示例要修改的数值

```cpp
constexpr double cabinet_height = 0.60;    // 柜顶离地
constexpr double support_height = 0.28;    // 中间支撑底部离地，约数需实测
constexpr double tcp_above_cabinet = 0.10; // TCP 高于柜顶多少
constexpr double ready_x = 0.20;          // 准备点 X
constexpr double ready_y = -0.40;         // 右臂准备点 Y
constexpr double approach_dx = 0.16;       // 沿机器人 +X 接近距离
constexpr double open_width = 0.07;        // 两指总开口
constexpr double grip_force = 1.0;         // 官方驱动力参数 N
```

以上均是待测量的示例值。目标 TCP 在 base_link 的 Z 为 `0.60-0.28+0.10=0.42 m`，即离地 70 cm；本例明确到达此高度，不沿用起点高度。TCP 高于柜顶不代表手指和整个工具都已越过柜顶，需结合实物工具尺寸和预览判断。接近方向是本体 +X，与夹爪朝向无关；抓取高度、方向和距离必须与瓶子位置相符。

第一段用 move_p 同时到达准备点和夹瓶朝向：本体坐标 (0.20,-0.40,0.42)，TCP RPY=(-90°,0°,0°)（已根据腕部相机位置翻转）。根据当前模型 TCP→法兰→夹爪固定安装变换，这对应夹爪水平向前（+X）、两指左右夹合（Y）。中途由规划器调整，不要求始终保持起点的朝下姿态。接近瓶子仍用 move_l 保持此朝向。需要改变目标姿态时，在同一个文件里修改 above 的 roll/pitch/yaw。

## 可写的任务接口

```cpp
auto p = robot.current_pose(); // 真实关节反馈 FK，单位 m/rad
p.x += 0.10;
robot.move_l(p);               // 直线；目标朝向与起点相同则沿途保持该朝向
robot.move_p(p);               // 点到点，约束终点位姿，中途朝向可能变化
robot.circle(0.03, "xy");       // 本体 XY 平面画圆，半径 3 cm
robot.gripper_open(0.07, 1.0);  // 总开口 7 cm，力参数 1.0 N
robot.gripper_close(1.0);      // 目标开口 0，力参数 1.0 N
```

可以在 task 中直接添加变量、循环、条件、自定义函数，不必拆成多个任务文件。底层 helper 是本项目对 MoveIt 的封装，继续复用原控制器、双臂桥接、官方 ROS 驱动与 SDK，不另外打开 CAN。

## 夹爪力的含义与完成条件

当前本地官方 `AgxGripperWrapper` 接口允许总开口 0..0.1 m、force 0.5..3.0，注释单位 N；实际行程还受实物夹爪配置限制。这里沿用该参数，不把它换算成百分比，也不保证等于实测瓶身接触力。1.0 是示例参数，不代表已经验证能夹稳满瓶水。

现有 ros2_control 轨迹控制器只控制位置。补充的力参数由任务在 do 确认后，通过 `/dual_joint_bridge/set_parameters` 设置所选臂的 `left_gripper_force` 或 `right_gripper_force`；桥接仅在该夹爪控制器执行期间，把该力放入官方 `control/joint_states` 的 effort 字段。官方驱动读取 width/force 并调用原有 SDK。未确认预览时既不设置力参数，也不下发运动。

张开动作要求真实开口在目标 ±2 mm 内。闭合遇到瓶子时无需达到零开口：控制器轨迹完成后，要求新鲜反馈下开口在 0.5 mm 范围稳定至少 0.5 s（最多等待 5 s）。这只是动作/稳定确认，**不是抓取成功或夹持力达标检测**；未使能、卡住等情况也可能表现为稳定。本例闭合后继续提起和搬运，因此下一段 do 前需结合现场确认夹持情况；接触、打滑和提瓶能力尚未自动验证。关闭控制输入通道不会发夹爪失能命令，程序结束不自动松开。

瓶子尚未加入场景，本例没有抓取物体附着或接触碰撞豁免；新增搬运预览检查的是机器人和场景，不包含所夹瓶子的外形。夹爪闭合动画是零开口，真实接触后可能停在瓶径处，RViz 实时模型使用实际反馈。

## 编译与运行

首次加入夹爪功能同时修改了桥接，需要编译两包，并手动退出原启动终端后重新启动一次：

```zsh
cd /home/a/Nero-7/hardware_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select nero_dual_bringup nero_official_commander_cpp --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
bash /home/a/Nero-7/scripts/start_dual_nero_moveit.sh
```

在另一个终端运行任务：

```zsh
bash /home/a/Nero-7/scripts/run_task.sh
```

以后只改并保存 robot_task.cpp，再运行 run_task.sh 即可：脚本会自动在 hardware_ws 编译任务包，成功后运行该目录下的可执行程序；编译失败直接退出，不运行旧程序。不必重启 MoveIt、驱动或 RViz。原 point_to_point.sh 也转到相同入口。

## 预览开关

顶部 `PREVIEW=true` 为默认：每段单独规划、预览。输入 `r` 回车可重新播放当前段（可重复），输入 `do` 回车执行后再进入下一段；其他输入/EOF 取消整个任务。`r` 只重新发布同一条已规划轨迹，不重新规划或发送运动；execute=false 的仅预览模式也支持 `r`。如果上一遍尚未播完，重播可能等待上一遍结束。柜子和工具使用模型碰撞检查。夹爪动作也单独预览。后段起点取前段完成后的实际反馈；这不是提前预演整个任务，也不承诺各段无停顿。

`PREVIEW=false` 跳过动画和 do，仍按顺序规划并执行。也可临时覆盖：

```zsh
bash /home/a/Nero-7/scripts/run_task.sh --ros-args -p preview:=false
```

目标和参数确认前保持默认预览。运行时 `arm:=left` 可切换臂，frame:=base_link 可设置坐标；切换坐标时必须重新换算绝对目标。旧 target_xyz/target_pose 覆盖只适合单个 move_p 测试，多步骤任务直接在文件中填写目标。

## 坐标与朝向

FRAME=base_link 原点在双臂中间支撑底部，+X 前、+Y 机器人左、+Z 上，左右臂共享；不是侧装的单臂 left_base_link/right_base_link。当前 world→base_link 只有 Z=0.28 m 平移，所以轴方向相同，绝对高度零点不同。

Pose 的 x/y/z/roll/pitch/yaw 都相对 FRAME，RPY 单位弧度，内部 R=Rz(yaw)Ry(pitch)Rx(roll)，不是三个机械臂关节角。可用 `p.yaw=radians(-30.0)` 设置绝对 yaw；单项 RPY 增量不等于任意绕世界轴做增量旋转。控制的是模型 TCP，实物工具尖端关系以安装模型为准。

## 本次验证范围

两包编译通过；桥接 9 项无 CAN 单元检查通过。独立 ROS_DOMAIN_ID=97 的 MoveIt/控制器/官方 gate/桥接模拟测试验证：预览取消零命令；两次 do 分别执行张开/闭合；力 1.7/.8 分别随正确夹爪指令传递；没有机械臂关节或另一侧指令；模拟瓶径 35 mm 阻止闭合仍可完成稳定确认；preview=false 无 stdin 也能运行。

报告：`/home/a/Nero-7/hardware_ws/validation/dual_moveit/gripper_report.json`。本次未执行实物；未验证满瓶夹取或真实夹持力。准备段后续通过了实物反馈起点上的只规划验证，仍须预览并结合现场确认。模拟栈现有点云 octomap 插件不可用，测试覆盖静态模型场景，不代表具备实时感知避障。

## 准备段规划失败修正

从实物读取 TCP 起点约 (0.0209,-0.2651,0.0112)，仅把 Z 改到 .42、保持原朝下姿态时，逆解查询返回 NO_IK_SOLUTION（开启/关闭碰撞检查均如此），直线路径完成比例约 .00244。改成水平夹瓶朝向但保留原 XY 时，逆解查询仍失败。改用明确准备点 (.20,-.40,.42)、TCP RPY=(90°,0°,0°) 后，无碰撞逆解与路径查询成功。以上均为模型计算，不是实物运动验证；不等于数学证明所有原目标解不存在。报告 lift_diagnosis.json / lift_grasp_candidates.json 位于 validation/dual_moveit。

## 修改后运行仍是旧距离

run_task.sh 固定读取 `/home/a/Nero-7/hardware_ws/install`。如果在 `/home/a/Nero-7` 执行 colcon，会生成根目录下的另一套 build/install，编译成功也不会更新启动脚本使用的程序。现在 run_task.sh 已自动调用固定目录的 build_task.sh，并直接运行 hardware_ws 下的可执行程序。从任意目录只需执行：

```bash
bash /home/a/Nero-7/scripts/run_task.sh
```

任务启动时打印已编入程序的 approach_dx（米）。此参数只影响第三段直线接近；第一段仍到达 ready_x/ready_y 指定的准备点。正在运行的旧进程不会随编译更新，需要退出该任务再运行。无需重启 MoveIt/RViz。

编译读取磁盘上已保存的 C++ 文件；编辑器未保存的内容不会参与编译。脚本传入的 ROS 参数原样传给新程序。

## 夹取后的搬运步骤

任务新增 lift_dz=.05、transfer_dx=.10、transfer_dy=.20、lower_dz=-.04 四个参数，单位米，均相对 base_link 的方向。先上抬5cm；再从实际反馈读起点，同时修改同一个目标的 x/y 并仅调用一次 move_l，构成 XY 斜向直线（长度约22.4cm，不是先Y再X）；接着下降4cm，最后打开。三段移动均保留各自起点朝向，每段包括夹爪张开都保留预览/do/r/取消。原 approach_dx=.16 与原准备点未改。新增全流程未执行实物；未建模瓶子的搬运外形。

## 斜向搬运 87% 完成率诊断

实物反馈起点约 (.38002,-.40009,.47005)，当前保持 TCP RPY≈(-90°,0°,0°) 时，目标 X+.20/Y+.33 即 (.58002,-.07009,.47005)。只读服务查询重复得到 fraction=.8704816；不检查碰撞仍相同；仅诊断关闭跳变过滤得到 .8756477，J2 距模型限位约 .93°。目标位姿碰撞开/关逆解查询均 NO_IK_SOLUTION。不能把残缺路径当作完整搬运执行。

同一起点、朝向、碰撞检查、跳变阈值下，候选 X+.15/Y+.33 的整段直线路径完成率1；原 X+.20/Y+.33 没有自动修改。候选能规划不等于实物安全验证，瓶子外形仍未加入模型。数据保存在 validation/dual_moveit/transfer_diagnosis.json。基础接口已增加每段 Cartesian 起点、目标和位移日志，便于明确失败段。

## 当前搬运位移：X+10cm / Y+20cm

按用户重新指定，transfer_dx=.10、transfer_dy=.20。以当前实物抬升后反馈为起点，本体坐标约 (.38002,-.40007,.47005)，目标为 (.48002,-.20007,.47005)，保持当前朝向、原碰撞检查与跳变阈值，整段直线完成率1；从该计算终点继续下降 .04 m 也完成率1。仅做规划查询，没有执行实物；此结果不包含瓶子外形碰撞验证。报告 validation/dual_moveit/transfer_10_20_report.json。此前20/33cm失败数据作为历史诊断保留。

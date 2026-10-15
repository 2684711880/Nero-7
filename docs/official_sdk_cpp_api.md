# 按官方 SDK 风格从 C++ 调用

## 跨语言方式

```text
                  同一套 C++ ArmCommander 接口
                              |
                         ROS 2 消息
                    /left|right/control/...
                         /            \
        仿真 official_command_bridge    官方 agx_arm_ctrl（Python）
                  |                           |
               MoveIt                    pyAgxArm SDK
                  |                           |
        JointTrajectoryController      CAN → 机械臂控制器
                  |
                Gazebo
```

不把Python嵌进C++，不在每个动作时启动Python子进程。
官方Python驱动是独立ROS节点；C++通过DDS传递标准消息，与语言无关。
官方源码保持原样，本项目统一函数名和消息契约。

## C++ 接口

定义在 `hardware_ws/src/nero_official_commander_cpp/include/nero_official_commander_cpp/arm_commander.hpp`。

| C++方法 | ROS topic后缀 | 消息 | 仿真执行 |
| --- | --- | --- | --- |
| move_j(7个角度) | control/move_j | sensor_msgs/JointState | 设置关节目标，MoveIt规划并执行 |
| move_p(末端位姿) | control/move_p | geometry_msgs/PoseStamped | 设置末端目标，MoveIt自由路径规划 |
| move_l(末端位姿) | control/move_l | geometry_msgs/PoseStamped | 从当前位置沿笛卡尔直线到目标 |
| move_c(起点,中间点,终点) | control/move_c | geometry_msgs/PoseArray | 三点圆弧采样、笛卡尔路径计算 |

`move_j`始终发送全部七个关节，名称joint1…joint7，单位rad，角度为绝对目标。
仿真接收端将它们映射到选定臂的left_joint1…或right_joint1…。
位姿可以传geometry_msgs::Pose，也可以传`ArmCommander::Pose6`：
`{x,y,z,roll,pitch,yaw}`，单位m/rad；C++层转换成四元数消息。

当前学习入口 `commander.cpp`：

```cpp
ArmCommander robot(node, "left", "base_link", send);
robot.move_j({0, 0, 0, 1.0, 0, 0, 0}); // 收臂
auto A = robot.currentPose();
auto B = A;
B.position.x += 0.15;
robot.move_l(B);                       // 向车头画15 cm直线
// robot.move_p(B);                    // 末端到点
// robot.move_c(A, M, B);              // 三点圆弧
```

圆弧不是完整圆，三点必须不同且不共线。当前仿真要求先到圆弧起点再调用move_c，
起点检查容差2 mm/0.01 rad。厂商控制器的起点接近、段间衔接等行为还需实机验证。

## 坐标和 TCP

`base_link`是本整车模型的车体坐标系，作为用户输入参考系。
所有位姿发送前先由TF转换到对应 `left_arm_base` / `right_arm_base`。
这一转换在C++层完成，因为官方Python回调不会根据frame_id执行TF变换。
当前测量/控制TCP默认是 `left_gripper_base` / `right_gripper_base`。

实际部署的TF名称不同，可以调用 `setFrames(control_frame, tcp_frame)`。
官方驱动的tcp_offset必须对应同一个真实TCP。不能把仿真夹爪基座与实机法兰/画笔尖混用。
命名空间默认为/left、/right，官方驱动也应使用同一namespace。

## 结果语义与实机边界

默认后端为Simulation，方法等待本项目control/result并返回CompletedInSimulation。
send=false返回DryRun，不发布运动命令。

显式构造Backend::OfficialDriver时直接对接官方消息接口，不等待本项目仿真结果topic。
它只返回PublishedToDriver，表示DDS确认了消息传递，不能视为驱动接受或运动完成。
本轮没有启动官方驱动、连接CAN或验证真实机械臂。正式串行实机任务还需要补全
反馈新鲜度、目标到达判定、故障/拒绝检测和超时处理，不能只切换backend就连续下发整套任务。

本项目当前结果消息为String，没有request_id，只支持一个调用者串行执行动作。
超时不会取消已经发布的命令，不自动重发。

## 仿真与实物的对应程度

统一的是函数、单位、目标坐标/TCP含义、消息结构和动作顺序。
仿真MoveIt与厂商固件不是同一个规划器；避障路径、逆解构型、速度曲线、圆弧衔接可能不同。
Gazebo模型、摩擦、负载和控制参数也影响效果。因此只能做到接口和任务语义对齐，
不能据此承诺轨迹细节、跟踪误差或动力学效果完全一致。

## 构建和运行

```zsh
cd /home/a/Nero-7
bash scripts/build.sh
bash scripts/build_official_control.sh
# 在相同ROS_DOMAIN_ID启动整车Gazebo（默认包含接收端）后：
export ROS_DOMAIN_ID=65
bash scripts/sim_commander.sh --ros-args -p send:=true
```

新增move_j/move_p接收接口后需重启仿真接收端。只修改C++动作代码时只需重编C++包。

## 验证

`test/verify_arm_api.py`在隔离ROS domain内运行C++ api_smoke的mock模式，
接收四类消息，调用真实官方Python回调，用mock SDK检查七关节顺序和位姿参数。
不会构造官方驱动节点或连接CAN。

`api_smoke`的sim模式依次执行move_j、move_p、move_l，再复位并执行move_c。
它属于BUILD_TESTING构建的测试程序，默认send=false；测试使用send=true。
报告见 `hardware_ws/validation/arm_api_report.json` 和
`sim_ws/validation/uniform_arm_api/`。

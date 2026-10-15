# NERO 官方控制代码核查

核查日期：2026-10-14。仅阅读源码，没有启动真实机械臂驱动或发送硬件命令。

## 来源版本

- 本工程已有官方仿真子模块 `agilexrobotics/agx_arm_sim`，固定提交
  `f8cd8b147c75d59e14f90fb0646770eefa268ed0`。
- 本次另行读取官方 ROS 驱动 `agilexrobotics/agx_arm_ros` 的 ros2 分支，提交
  `bc485ab8e3d25abbcd3a1b58c105be12bad5dadc`。
- 本次另行读取官方 Python SDK `agilexrobotics/pyAgxArm`，提交
  `1ae40ad4755ca25ad32bece3ab42325698185fc7`。

ROS 和 SDK 已固定版本正式引入 `third_party/agx_arm_ros`、
`third_party/pyAgxArm`。SDK安装于项目独立环境 `.venv-official`，
官方控制与消息包在独立 `hardware_ws` 构建，没有加入整车仿真运行依赖。
以下结论针对这些提交，不能据此推断用户实机的固件版本或行为。

## 官方已经有的内容

| 内容 | 官方实现位置 | 可复用方式 |
| --- | --- | --- |
| 直线运动 | SDK `pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py` 的 `move_l()` | 真实机械臂发送目标法兰位姿；SDK编码并发送指令 |
| 圆弧运动 | 同文件 `move_c(start_pose, mid_pose, end_pose)` | 三点定义圆弧；不是任意完整圆接口 |
| 关节到点 | 同文件 `move_j()` | 复用关节运动模式 |
| 末端到点 | 同文件 `move_p()` | 复用位姿到点运动 |
| 跟随关节目标 | 同文件 `move_js()` | 官方文档说明不做平滑或轨迹规划，不能当普通到点接口使用 |
| ROS控制接口 | ROS驱动 `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py` | C++应用可通过ROS消息调用，不要求把Python SDK改写为C++ |
| 直线与圆弧输入样例 | ROS驱动 `test/nero/test_move_l.yaml`、`test_move_c.yaml` | 复用消息结构，坐标必须按当前机械臂实际状态修改 |
| SDK调用样例 | SDK `pyAgxArm/demos/nero/test1.py` | 复用调用流程；示例等待完成逻辑还需完善 |
| Gazebo与MoveIt配置 | 本工程官方子模块 `Moveit2/nero_gripper_moveit_config` | 模型、IK、关节限制、轨迹控制器配置已具备 |

所检查的官方仿真包和ROS驱动主仓库中，没有找到可直接复制的C++画整圆/直线应用类。
已有的是配置、Python驱动/SDK和测试输入。C++可以保留为我们自己的应用层。

## 真实机械臂命令链

```text
C++应用发布目标TCP位姿
  → control/move_l (PoseStamped) 或 control/move_c (PoseArray)
  → 官方 _move_l_callback / _move_c_callback
  → _check_can_control：启动就绪、反馈、使能、控制门控
  → _create_pose_cmd：四元数转RPY，并按tcp_offset转成法兰位姿
  → SDK move_l / move_c
  → 编码CAN位姿与运动模式命令
  → 机械臂控制器

机械臂反馈 → SDK → feedback/joint_states、feedback/tcp_pose、feedback/arm_status 等
```

ROS驱动话题名是相对名称，已有namespace启动参数，可供左右臂分别实例化。
SDK的原始位姿接口用 `[x,y,z,roll,pitch,yaw]`，单位m/rad。
ROS位姿接口用geometry_msgs四元数并处理TCP偏移。

所检查的ROS回调只读取 `msg.pose` 或 `msg.poses`，没有按 `header.frame_id` 做TF变换。
因此不能把整车odom坐标下的点原样发给单臂官方接口；应用适配层必须先转换到对应臂的
控制坐标系，并核对官方基坐标系方向与模型。仅填写frame_id不能实现坐标变换。

SDK `move_l` / `move_c` 的可见实现负责模式与消息发送，没有在Python里生成密集轨迹点。
结合官方API对运动模式的说明，规划/平滑能力由机械臂控制器承担；具体算法和实际精度
没有从这些源码得到证明。`move_c`发送三组位姿及编号1/2/3，不能假定重合起终点可确定整圆。

## 仿真执行链与官方MoveIt实机接法

本工程的官方Gazebo配置使用 `JointTrajectoryController` 与Gazebo硬件接口。
它不经过CAN SDK，不消费上述 `control/move_l` / `control/move_c` 话题。

本次检查的官方ROS MoveIt配置中，ros2_control使用 `mock_components/GenericSystem`，
并将其joint_states输出重映射到control_topic；官方驱动订阅这个topic，按fast_mode选择
`move_js`或`move_j`。这是命令转发结构。不能把mock轨迹执行成功直接当成真实机械臂的
轨迹跟踪成功，也不能把配置里的200 Hz当作测得的CAN或电机闭环频率。

## 在官方代码上增加的最小内容

1. 保留官方模型、控制器、ROS驱动、SDK协议与固件适配；不自行重写CAN协议。
2. C++应用层组织左右臂目标和任务参数；真实机械臂路径可复用官方直线与三点圆弧接口。
3. 增加坐标转换、真实TCP定义和左右臂namespace适配。
4. 在发送成功之外，增加反馈新鲜度、到达位置/姿态容差、超时与任务结果。
   官方SDK示例只延时后轮询motion_status==0，不能单凭该状态证明到达目标。
5. 整圆若采用多段圆弧，需先验证控制器对段间衔接、停顿和姿态的行为；不能声称已实现平滑整圆。
6. Gazebo继续复用已有官方MoveIt/控制器链；若要求同一上层任务同时支持SDK实机与Gazebo，
   在应用层提供两种执行适配。二者需分别验证，不把SDK命令直接接到Gazebo关节控制器。

当前已完成源码引入及C++消息接口接入；没有启动实机驱动。
工作空间和运行方法见 `hardware_ws/README.md`，测试使用mock SDK，尚无真实运动验证。

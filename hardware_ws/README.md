# 官方 NERO 控制代码接入

最新C++学习入口使用官方风格的 `move_j`、`move_p`、`move_l`、`move_c`，
见 [统一调用接口与跨语言说明](../docs/official_sdk_cpp_api.md)。
仿真接收端现在支持四类消息；实机仅验证到官方回调的mock SDK接口，未执行实机运动。

当前 `commander.cpp` 已简化为仅从当前A点到B点画直线，使用方法见
[最小直线程序](../docs/cpp_straight_line_first.md)。下文原有完整参数示例改用
`commander_reference`，方便后续对照。

本工作空间使用官方 ROS 驱动、官方 SDK，加一层 C++ 消息调用。
不包含自行实现的 IK、轨迹插补或 CAN 协议，不启动现有整车 Gazebo。

在Gazebo中使用同一个C++发布入口的方法见 [C++话题控制仿真](../docs/cpp_topic_simulation.md)。
仿真由mobile_dual_nero中的接收端执行，支持官方直线/圆弧消息格式；named_target和
wait_result是本项目仿真扩展，不能直接当作官方实机接口使用。

## 目录与源码入口

- `../third_party/agx_arm_ros`：固定版本的官方 ROS 仓库，保持上游源码原样。
- `../third_party/pyAgxArm`：固定版本的官方 Python SDK。
- `src/agx_arm_ctrl`、`src/agx_arm_msgs`：到官方 ROS 包的相对符号链接。
- `src/nero_official_commander_cpp/src/commander.cpp`：我们的 C++ 调用层。

两个工作空间分别构建和source，避免官方同名模型包与仿真工作空间冲突。
本次只构建官方控制、消息包和 C++ 程序，未引入官方单臂 MoveIt 模型替换整车双臂模型。

## 构建

使用已有 ROS 2 Humble 系统环境，在全新终端执行：

```zsh
cd /home/a/Nero-7
bash scripts/build_official_control.sh
source scripts/official_control_env.sh
```

脚本初始化固定版本子模块，用 `.venv-official` 安装 SDK 及其 Python 依赖，
然后构建工作空间。SDK 环境允许读取系统 scipy/numpy 和 ROS Python 包，
不修改系统 Python 安装。环境脚本同时支持 bash/zsh，不启动任何驱动。
新机器需先具备 ROS 2 Humble、colcon、python3-venv、numpy、scipy 等开发依赖。

## 先看官方怎样写

1. 官方 `agx_arm_ros/test/nero/test_move_l.yaml`：目标TCP位姿消息。
2. 官方 `src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`：
   `_move_l_callback()` / `_move_c_callback()` → `_create_pose_cmd()` → SDK。
3. SDK `pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py`：
   `move_l()` / `move_c()` → 模式设置与CAN指令。
4. SDK `pyAgxArm/demos/nero/test1.py`：调用示例。

## C++ 调用，默认只检查输入

```zsh
ros2 run nero_official_commander_cpp commander_reference --ros-args \
  --params-file hardware_ws/src/nero_official_commander_cpp/config/line.yaml \
  -r __ns:=/left

ros2 run nero_official_commander_cpp commander_reference --ros-args \
  --params-file hardware_ws/src/nero_official_commander_cpp/config/arc.yaml \
  -r __ns:=/right
```

默认 `send=false`，不发布控制消息。参数 `poses` 每个姿态为
`[x,y,z,qx,qy,qz,qw]`；直线1组，圆弧3组，位置单位m。
输入例子来自官方测试yaml，只示范消息格式，未验证为当前实机可执行目标。
`send=true` 才会等待订阅者并发布一次，分别对应
`/<namespace>/control/move_l`（PoseStamped）和 `control/move_c`（PoseArray）。
DDS确认只代表消息传递，不代表驱动接受、运动完成或到达目标。

官方位姿回调不依据frame_id执行TF变换。坐标必须已经转换到对应机械臂基坐标系，
TCP必须与驱动的tcp_offset一致；填写frame_id不会改变坐标。
圆弧是三个不同且不共线的点，不能用重合起终点表示完整圆。

## 验证方式

```zsh
export ROS_DOMAIN_ID=64
python3 hardware_ws/src/nero_official_commander_cpp/test/verify_official_bridge.py
```

测试使用独立namespace的ROS订阅者，验证C++默认不发送、显式发送的消息类型与内容、
无效四元数/退化圆弧拒绝发送，再调用真实官方回调并以mock SDK接收结果。
不会构造官方驱动节点，也不会连接CAN。测试结果只证明接口链，未验证实机运动、
完整圆、多段衔接、到达反馈和双臂同步。

2026-10-14 已构建三个包，以上七项接口检查全部通过，标准错误输出为空。
记录见 `validation/bridge_report.json`。项目独立环境使用 NumPy 1.26.4、SciPy 1.13.1、
python-can 4.6.1，依赖版本记录在 `../deps/official_control_requirements.txt`。

当前尚未启动真实驱动。后续接实机需先读取实际CAN、固件、TCP及反馈，
再使用官方启动文件；官方默认auto_enable=true，不能把启动当作只读检查。

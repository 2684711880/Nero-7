> 当前统一入口为 `robot_task.cpp`，整个流程写在 task(robot) 中；完整说明见 robot_task.md。

# 点到点：统一任务入口

修改文件：`hardware_ws/src/nero_official_commander_cpp/src/robot_task.cpp`。

```cpp
auto point = robot.current_pose(); // 当前位置及朝向
point.x = 0.20;
point.y = -0.40;
point.z = 0.60;
// point.roll = 0.0;  // 需要改变朝向时填写，单位弧度
// point.pitch = 0.0;
// point.yaw = 0.0;
robot.move_p(point);
```

XYZ 是 world 下的绝对位置，单位 m；默认沿用当前末端朝向。目标值为可编辑示例，需结合现场和预览选择。TCP 为当前模型的 right/left_tcp_link。

## 运行

保持现有实物 MoveIt/RViz 运行。默认右臂，先预览，输入 do 执行；Enter 取消：

```zsh
bash /home/a/Nero-7/scripts/point_to_point.sh
```

关闭预览，直接按代码目标规划并执行，不等待 do：

```zsh
bash /home/a/Nero-7/scripts/point_to_point.sh --ros-args -p preview:=false
```

选择左臂：追加 `--ros-args -p arm:=left`。只预览、禁止执行：`--ros-args -p execute:=false`。关闭预览不关闭 MoveIt 规划、模型碰撞检查、反馈检查及 RViz 执行后的状态刷新。

临时指定目标、无需编译（保留旧命令兼容）：

```zsh
bash /home/a/Nero-7/scripts/point_to_point.sh --ros-args \
  -p arm:=right -p target_xyz:='[0.20, -0.40, 0.60]'
```

完整位姿覆盖用 `target_pose:='[x,y,z,roll,pitch,yaw]'`，m/rad。若同时提供，两者以 target_pose 为准。命令行覆盖用于当前单目标示例；将来写多目标顺序任务时不传这些全局覆盖参数，直接在源码中逐点调用 move_p。

## 编译

```zsh
cd /home/a/Nero-7/hardware_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select nero_official_commander_cpp
```

只需重新运行任务程序，机械臂驱动和 RViz 不用重启。

## 辅助实现

`include/nero_official_commander_cpp/moveit_robot.hpp` 负责节点运行、反馈、MoveIt 规划、预览确认、执行同一份轨迹和 RViz 同步。任务文件不再混入直线/画圈/关节目标模式。

这里的 robot.move_p 是本项目对官方 MoveIt 流程的封装，不是直接调用 SDK 的同名固件命令。直接 SDK move_p 不经过当前 MoveIt 柜体场景，也不能保证走刚才预览的轨迹。官方驱动、joint_bridge 和控制器保持原有实现。

模型中的柜体、安装高度和工具外形应对应现场；预览和模型碰撞通过不是实物安全证明。本次测试使用不连接 CAN 的模拟反馈，不自主执行实物。

已编译并通过无 CAN 模拟反馈回归：预览取消时 0 条运动指令；输入 do 后仅右臂执行、终点反馈确认及 RViz 同步；preview=false 在 stdin 关闭时仍完成规划执行且不输出预览/确认提示。日志见 hardware_ws/validation/dual_moveit/simple_ptp_direct.log，预览执行报告见 point_to_point_report.json。本次没有操作实物。

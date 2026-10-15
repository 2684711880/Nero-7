# 第一步：先收臂，再从 A 到 B 画直线

最新入口已改为与官方SDK同名的 `ArmCommander::move_j/move_p/move_l/move_c`，
见 [官方风格C++接口](official_sdk_cpp_api.md)。下面的goToStart/drawLine是保留的旧版教学说明；
当前commander.cpp用move_j显式写七个角度收臂，再用move_l画直线。

现在只读/修改 `hardware_ws/src/nero_official_commander_cpp/src/commander.cpp`。
圆弧和复杂参数分支已移出主入口，旧版保存在 `commander_reference.cpp`。
发布和等待结果的辅助代码在 `include/nero_official_commander_cpp/line_commander.hpp`，
暂时不需要修改。仿真接收端仍使用现有 `official_command_bridge`。

## 坐标系

```text
odom                         地面上的里程计参考系；当前MoveIt规划参考系
  └─ base_link               小车车体坐标系
      └─ arm_mount_plate
          └─ dual_nero_base_link    双臂躯干模型基座
              ├─ … → left_arm_base  左臂基座 → 各关节 → left_gripper_base
              └─ … → right_arm_base 右臂基座 → 各关节 → right_gripper_base
```

省略号表示中间还有固定连接，不是两个link直接相连。
末端位姿需要同时指定“被测link”和“参考frame”。
例如 `base_link → left_gripper_base` 描述左夹爪基座相对于车体的位姿；
`left_arm_base → left_gripper_base` 才描述它相对于左臂基座的位姿。
末端自身坐标系不是填入x/y/z时默认采用的参考坐标系。

这版明确设置 `frame="base_link"`。按模型车体坐标轴，X向前、Y向左、Z向上，
所以 `B.position.z -= 0.01` 是相对车体向下1 cm，不是沿末端自身-Z移动。
车体默认在平地保持直立时，车体向下与世界向下一致；底盘移动后base_link仍随车体移动。
也可改为 `left_arm_base`；此时同样的数字对应左臂基座的轴向，不能假定它与车体坐标轴同向。

当前控制点是 `left_gripper_base`（夹爪基座），不是夹爪指尖或画笔尖。
姿态四元数也相对于同一个frame。`auto B=A`使B复制A的姿态，当前只修改位置。

## A、B 与执行

```cpp
commander.goToStart("tucked", send); // 先收臂，完成后再读A
auto A = commander.currentPose(); // 当前末端在base_link下的实际位姿
auto B = A;
B.position.x += 0.15;            // 相对A，沿车体+X向前移15 cm
commander.drawLine(A, B, send);
```

要给B绝对坐标，直接改成 `B.position.x=...; B.position.y=...; B.position.z=...;`，
单位m，仍相对于base_link。不是先把末端设为局部原点再填这些数。

此版本先通过命名关节姿态tucked收臂（joint4=1.0 rad，其余关节=0），以收臂后的当前末端为A。
这段准备动作由MoveIt规划，不要求末端走直线。随后`move_l`消息只发送B，接收端从当前反馈规划到B。
不能任意修改A就以为机械臂会先到A：辅助代码会检查当前是否已经位于A附近，
不匹配就拒绝发送。任意A→B任务还需要先完成“到A”的单独动作，之后再画A→B。

## 编译运行

```zsh
cd /home/a/Nero-7/hardware_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select nero_official_commander_cpp
```

先按整车README启动仿真（默认包含接收端），然后另一个终端运行：

```zsh
cd /home/a/Nero-7
export ROS_DOMAIN_ID=65  # 与仿真使用相同domain；若仿真用0，这里也用0
bash scripts/sim_commander.sh                    # 打印A/B，不运动
bash scripts/sim_commander.sh --ros-args -p send:=true  # 发送并等待执行结果
```

修改并编译此文件后，重新运行发布程序即可，仿真接收端不用重启。
重复运行会先回到tucked，再读取A并向前画15 cm，不会从上次终点继续累加15 cm。
send=false不执行收臂，打印的A/B基于当前实际位置，只用于检查代码。
新增tucked姿态后必须重建仿真配置并重启MoveIt/接收端；仅修改后续B点时无需重启。

旧版参考入口仅用于回看和回归测试：`ros2 run nero_official_commander_cpp commander_reference`。
之前的poses/relative/offsets/motion等参数属于参考入口，不再控制现在的最小commander。

2026-10-14 已构建并完成Gazebo验证：默认运行只打印A/B；显式send=true完成
base_link下向下1 cm的直线，控制器返回成功，终点关节误差约0.000044 rad。
记录位于 `sim_ws/validation/minimal_line/`，未执行实机运动。

随后已验证较大幅度版本：左臂tucked → A约(-0.002696,0.714147,0.606199)m
→ B约(0.147304,0.714147,0.606199)m，沿+X画15 cm。准备和直线均返回成功。
记录位于 `sim_ws/validation/tucked_line/`。本轮仅验证左臂、空场景仿真，不能推断实机或其他环境。

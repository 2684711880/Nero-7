# 先收臂，再在 X–Y 平面画圆

显示绿色规划路径和橙色反馈轨迹的方法见 [RViz轨迹显示](trajectory_display.md)。

当前可编辑入口：`hardware_ws/src/nero_official_commander_cpp/src/commander.cpp`。
圆弧接口定义：同包 `include/nero_official_commander_cpp/arm_commander.hpp`。

## 按主文件顺序读

1. `frame="base_link"`：位置和姿态相对车体坐标系，X向车头、Y向车体左侧、Z向上。
2. `move_j({0,0,0,1,0,0,0})`：先到收臂姿态，各关节是绝对角度，单位rad。
3. `A=currentPose()`：等待收臂完成后，读取实际末端位姿作为圆起点。
4. `radius=0.05`：半径5 cm、直径10 cm。
5. `O=A; O.x+=radius`：圆心在A的+X方向，A位于圆的-X端。
6. 从O复制M1/B/M2，只改x/y；所有点的z和姿态都来自A。
7. 两次move_c依次执行，回到A，一圈完成。

相对于圆心O，各点位置为：

| 点 | X偏移 | Y偏移 | Z偏移 |
| --- | --- | --- | --- |
| A | -r | 0 | 0 |
| M1 | 0 | +r | 0 |
| B | +r | 0 | 0 |
| M2 | 0 | -r | 0 |

从+Z朝XY平面看：

```text
                  +Y
                   ↑
                   M1
                ╭─────╮
              A │  O  │ B   → +X
                ╰─────╯
                   M2
```

```cpp
robot.move_c(A, M1, B); // 经过+Y侧的半圆
robot.move_c(B, M2, A); // 经过-Y侧的半圆，返回起点
```

`move_c`三个参数是“起点、圆弧中间点、终点”。中间点用于确定圆，不是先走一条直线到中间点。
三个点不能共线，也不能把同一个点同时作为起点/终点来表达完整圆，所以用两个半圆。
O只是计算几何用，不会被作为目标发给机械臂。

两段分别规划并执行，在B处会减速/停顿；几何上是整圆，目前不保证连续恒速。
控制点仍是left_gripper_base，不是画笔尖。当前底盘保持停止。

## 运行

新建终端启动已有的仓库场景仿真（ROS_DOMAIN_ID与发布程序一致），接收端默认随仿真启动。
然后在另一个终端：

```zsh
cd /home/a/Nero-7
export ROS_DOMAIN_ID=65
bash scripts/sim_commander.sh --ros-args -p send:=true
```

默认send=false只打印，不执行收臂和画圆；此时打印圆心基于当前位姿，不能当成收臂后位姿。
只修改radius或圆点构造后，重编C++包即可：

```zsh
cd /home/a/Nero-7/hardware_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select nero_official_commander_cpp
```

## 验证范围

2026-10-15：当前整车warehouse场景、左臂、半径5 cm、速度和加速度缩放0.8。
收臂及两个半圆执行成功，记录373个不同时间戳的TCP TF样本。
采样最大径向误差约0.079 mm，平面高度误差约0.057 mm，闭合误差约0.010 mm，
观测行程约0.31413 m，理论周长约0.31416 m。
这是仿真TF采样结果，不是连续时间误差上界或实机精度，其他半径需重新验证。

日志及样本：`sim_ws/validation/xy_circle/`。
复验：同domain启动仿真后，source ROS和sim_ws环境，运行 `python3 scripts/verify_xy_circle.py`。

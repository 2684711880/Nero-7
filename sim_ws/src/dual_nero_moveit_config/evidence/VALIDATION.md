# 验收记录（2026-10-14）

Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11.10.2 / MoveIt 2.5.9。

- xacro 展开和 check_urdf 通过，所有 mesh 文件存在。
- colcon 构建通过。
- Gazebo 中模型生成成功；五个控制器 active。
- 完整 Gazebo + RViz 启动后，自动验收 12 项通过：10 次轨迹规划执行，2 次末端位姿规划。
- 臂关节最大反馈误差：0.000134 rad。
- 夹爪最大反馈误差：0.000087 m。
- SRDF 无左侧与右侧之间的碰撞豁免；本次初始状态碰撞检查通过。

验证程序：src/verify_dual.cpp；原始输出：evidence/verify_dual.log。
仅证明上述有限目标的规划/执行链路通过，未证明整个工作空间可达、接触动力学准确或实机适用。
相机只显示模型，因此当前不启用 Octomap/物体识别。

```text
[INFO] [1791944256.751875875] [verify_dual]: PASS left_arm -> ready: 27 points, max joint error 0.000091 rad
[INFO] [1791944259.322955808] [verify_dual]: PASS left_arm -> home: 26 points, max joint error 0.000086 rad
[INFO] [1791944259.350718169] [verify_dual]: PASS left_arm IK pose planning: 22 points
[INFO] [1791944262.742412453] [verify_dual]: PASS right_arm -> ready: 27 points, max joint error 0.000102 rad
[INFO] [1791944265.312390654] [verify_dual]: PASS right_arm -> home: 27 points, max joint error 0.000097 rad
[INFO] [1791944265.344479178] [verify_dual]: PASS right_arm IK pose planning: 59 points
[INFO] [1791944268.696607004] [verify_dual]: PASS both_arms -> ready: 27 points, max joint error 0.000091 rad
[INFO] [1791944271.269616164] [verify_dual]: PASS both_arms -> home: 27 points, max joint error 0.000134 rad
[INFO] [1791944273.604875028] [verify_dual]: PASS left_gripper -> open: 17 points, max joint error 0.000080 m
[INFO] [1791944276.186494527] [verify_dual]: PASS left_gripper -> close: 26 points, max joint error 0.000087 m
[INFO] [1791944278.516438047] [verify_dual]: PASS right_gripper -> open: 16 points, max joint error 0.000042 m
[INFO] [1791944281.096268170] [verify_dual]: PASS right_gripper -> close: 26 points, max joint error 0.000077 m
```

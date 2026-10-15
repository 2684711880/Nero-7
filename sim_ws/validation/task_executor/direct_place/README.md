# 放置直接下降修正

原因：旧任务在 b2 到位后下发 r6 move_p 到上方位姿，给自由关节空间重规划留下额外绕行机会。
修正：删除 r6，r7 move_l 直接依赖 b2，随后 r8 松开。其余动作保持。任务编号 bottle_transfer_002，右臂共七步。

在独立 ROS_DOMAIN_ID=77、GAZEBO_MASTER_URI=http://127.0.0.1:11346 的重置场景完成抓放，未修改用户正在运行的场景。

验收：任务 SUCCEEDED；水瓶水平误差 0.00681 m，竖直程度 0.999999998。下降阶段各关节角度跨度最大 0.335 rad，未出现整圈旋转。完整采样见 report.json。

回归：修正前场景契约测试失败（放置序列为 move_p、move_l、gripper_open），修正后为 move_l、gripper_open。20 项单元/回归测试及 8 项 ROS 通信恢复检查通过。

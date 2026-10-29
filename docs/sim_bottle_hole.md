# 固定中间相机与右臂持瓶戳孔：仿真

独立于旧的右腕 D405 持杆场景。左臂固定夹板，中间模拟深度相机检测孔，右臂从侧面夹横放的瓶身，瓶底朝前；右腕相机紧贴腕部，放在瓶口一侧（与瓶底推进方向相反）；瓶底方向已翻转到夹爪局部 -X。Gazebo/MoveIt 的瓶子带碰撞体，瓶子与夹爪是固定连接，不模拟夹持摩擦或瓶子变形。相机安装位姿、瓶子形状及支架均为仿真假设。

示例尺寸：瓶身直径 56 mm、长 150 mm；瓶颈直径 30 mm、长 40 mm；瓶轴垂直于夹爪伸出方向，夹持位置沿瓶轴偏向瓶口。修改 `sim_ws/src/dual_nero_moveit_config/launch/bottle_hole_scene.py` 中的四个尺寸或固定位置后重新构建，规划器从仿真 TF 读取瓶底工具点。板孔名义直径 90 mm。

```bash
cd /home/a/Nero-7
source /opt/ros/humble/setup.bash
colcon --log-base sim_ws/log build --base-paths sim_ws/src --build-base sim_ws/build --install-base sim_ws/install --packages-select dual_nero_moveit_config
bash scripts/sim_bottle.sh start
```

新终端，查看中间相机实时深度检测（`q` 退出）：

```bash
cd /home/a/Nero-7
bash scripts/sim_bottle.sh detect
```

另一个终端，采集深度、规划并在 RViz 中预览；再执行仿真：

```bash
cd /home/a/Nero-7
bash scripts/sim_bottle.sh plan
bash scripts/sim_bottle.sh execute
```

`camera` 显示中间相机彩色画面，`stop` 结束本场景。启动前退出其他 Gazebo。计划、检测和执行记录保存在 `sim_ws/validation/bottle_hole/latest/`；每次执行前重新检测规划。暂未实现瓶子专用回退。用户摆动左臂后需要重新检测和规划，且孔要完整处于相机视野内。

在默认 `hole_ready` 姿态，中间模拟深度相机检测到孔，当前示例瓶身直径调至 56 mm 后，起始状态和到孔前 10 mm 的规划通过碰撞检查；仿真执行的瓶底中心横向误差约 4.2 mm，距板中面约 13.6 mm。该结果不表示已戳入。

旧的 60 mm 示例瓶身与右腕相机支架在瓶口根部的碰撞网格存在约 0.6 mm 交叠；现在的示例尺寸及安装位置仍需与实物核对。保存的 336L 标定是左臂局部基座到彩色光学系，真实深度坐标还需正确转换，并验证与右臂基座、瓶底 TCP 和改变后腕相机的碰撞关系。

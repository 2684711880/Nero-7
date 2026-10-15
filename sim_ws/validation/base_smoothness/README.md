# 组合移动顿挫诊断与修复（2026-10-15）

同一输入 `bottle_transfer_combined.json`，每轮从重置的默认仓库场景开始；独立 ROS_DOMAIN_ID=79 / Gazebo Master 11347，单套执行节点、无 Gazebo 图形界面。没有连接真实硬件，夹持为仿真 checked_fixed_joint。

## 诊断结论

无界面也复现了真实车体速度反复接近零。基线各段仿真时间/墙钟时间约 0.999，采样中的最大命令年龄约 21 ms、关节反馈年龄约 9 ms，没有整体仿真变慢或长时间失联的证据。基线停轮采样中，约 97%～99% 与较大转向误差一致。当前证据支持控制策略导致的顿挫；没有测量 GUI 帧率，不能据此评价显示流畅度。

三处相互作用：

1. 底层只要任一轮转向误差超过 0.12 rad，就把四轮轮速同时置零。连续转向反复触发这条硬门槛。
2. 回程的旋转中心接近某个轮子；该轮目标速度很小，目标方向却可快速变化。基线用同样权重对待它和高速轮，把低速轮的转向变化扩大成整车停轮。
3. 上层停靠用的最低驱动补偿也参与了巡航：在误差 8 mm / 0.008 rad 处突然启用，并且原角速度补偿按位置误差定符号，可能反转前馈加反馈后的角速度。这会改变平移/转向比例，进一步激发轮子重新对齐。

## 修改

- AIR 控制器：按各轮目标轮速相对最大轮速加权转向误差；有效误差 0.12～0.5 rad 之间用 smoothstep 同比例降低四轮轮速，达到 0.5 rad 仍停止驱动。零速命令与反馈超时仍停轮。
- C++ 位姿控制：最低驱动补偿随误差连续变化，保持合成速度符号，合成速度经过零时连续退去；参考速度较高时退出停靠补偿，接近参考停止时平滑恢复。参考停止且误差超出内层到位容差时仍保留低速驱动力。
- 没有修改任务 JSON 的目标、机械臂动作、依赖关系或原分段 `base_move_to` 的实现。

## 对照结果

[速度曲线](comparison.png) 中蓝色为 Gazebo 实测平移速度，橙色为实测角速度，红色阴影为请求运动时四轮轮速命令为零。起步对齐和到位微调仍存在；巡航中反复停轮已经消除。

| 指标 | 基线 | 修复后 |
| --- | ---: | ---: |
| b2 搬运中段停轮占比 | 15.48% | 0% |
| b3 回程中段停轮占比 | 36.14% | 0% |
| b2 / b3 中段进入停轮次数 | 16 / 25 | 0 / 0 |
| b2 / b3 实际速度接近零占比 | 3.92% / 5.96% | 0% / 0% |
| b2 / b3 执行时间 | 59.46 / 47.46 s | 49.85 / 47.04 s |

中段定义为各底盘动作时间的中间 60%；停轮占比在同时请求平移 >0.01 m/s、旋转 >0.03 rad/s 的采样中计算。实际近零要求平移和旋转均 <0.003，使用该中段的全部样本。此处百分比为一次完整基线与一次最终修复运行的测量，未作跨场景统计保证。

[自动比较结果](comparison.json) 全部通过；[完整搬运验收](final_bottle.json) 通过：放瓶水平误差约 10.4 mm，底盘返回位置误差约 5.2 mm、朝向误差约 0.0147 rad，左臂 J2 到达 +60°，J1 未明显改变。

[Gazebo 取消测试](cancel_report.json) 通过：运动中取消后后续动作 BLOCKED，实际制动位移约 1.8 mm。[DDS 协议检查](protocol/report.json) 六项通过，包含取消、超时、失效里程计与停止未确认后的锁定。

[原分段夹水流程回归](legacy_bottle.json) 也通过：放瓶水平误差约 2.9 mm，底盘返回位置误差约 2.8 mm、朝向误差约 0.0213 rad。该任务使用原 `base_move_to`，确认共享 AIR 控制器的修改可用于旧流程。

59 项单元测试通过。其中先重现后修复的测试覆盖：0.12 rad 门槛附近轮速突降、巡航角速度意外反向、补偿不连续、前馈/反馈抵消，以及实际记录的近旋转中心低速轮拖停整车场景。

原始诊断记录保留为 `baseline.json`；`fixed.json`、`compensation_only.json`、`weighted_only.json` 为逐项修改时的中间对照；`final.json` 为最终版本。记录器不发送控制命令。

## 复测

在独立、重置的仿真场景和单套适配器中启动记录器，然后另一个终端提交同一任务。所有终端须加载同一 ROS 环境并使用同一 ROS_DOMAIN_ID。

```zsh
python3 src/nero_task_executor/tests/observe_base_smoothness.py \
  --output /tmp/smoothness.json
```

同时使用 `observe_sim.py --task-id bottle_transfer_combined_001 --return-home --expect-combined --output /tmp/bottle.json` 观测放瓶和返回结果。自动对照：

```zsh
python3 src/nero_task_executor/tests/compare_base_smoothness.py \
  --baseline validation/base_smoothness/baseline.json \
  --candidate /tmp/smoothness.json --bottle /tmp/bottle.json \
  --output /tmp/comparison.json
```

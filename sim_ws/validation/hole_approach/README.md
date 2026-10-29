# 方板穿孔：历史仿真结果

| 场景 | 横向误差 radial_error_mm | 板中面法向距离 signed_plane_offset_mm |
| --- | --- | --- |
| square_default | 2.4807 | 26.0058 |
| square_offset_j1_003 | 2.3556 | 25.9681 |

每个目录只发布检测图、检测参数和执行结果，不发布可重放的关节轨迹。数字来自对应 `execution.json`，仅为当时 Gazebo 模型结果。深度检测 `center_error_mm` 与模型板中面比较，包含约 4 mm 的表面到中面偏移。

`latest/`、`live/` 是运行时输出，已忽略；旧圆板 `first_loop/` 留在本地，不用于当前方板验收。操作见 `docs/sim_hole_approach.md`。

# NERO 工程整理盘点（2026-10-28）

大小为盘点时的近似值。随后已将根目录标定样本归档至 `archive/handeye/`，并更新当前配置引用；未删除样本或停止任何程序。

| 内容 | 作用与处理建议 |
| --- | --- |
| `hardware_ws/src/`、`scripts/`、`config/` | 实机控制、手眼标定和配置；保留。部分文件未被 Git 跟踪。 |
| `sim_ws/src/`（约 1.2 GB） | 仿真与场景源码及资源；是否仍需仿真应由使用者决定。 |
| `third_party/`（约 577 MB） | 官方 SDK、机器人模型、重力补偿及 ICP 依赖；不能按目录名判定为无用。 |
| `.venv-gravity/`（约 613 MB） | 官方重力补偿的 Python 环境；当前右臂补偿进程正在使用。 |
| `.venv-official/`（约 229 MB） | SDK、手眼及实机工具 Python 环境；保留。 |
| `hardware_ws/install/`（约 387 MB） | 实机启动所依赖的已编译包；删除会使相关命令不可用，直到重新构建。 |
| `hardware_ws/build/`（约 166 MB） | 编译中间文件；通常可重建，但清理前应确认构建环境及不需要其诊断信息。 |
| `sim_ws/build/`（约 118 MB）、`sim_ws/install/`（约 104 MB） | 仿真构建与运行产物；删除 install 会要求重新构建仿真。 |
| 根目录 `build/`（约 145 MB）、`install/`（约 44 MB） | 根目录构建产物；尚需确认是否有外部脚本引用，勿直接认定为无用。 |
| `calibration_ws/`（约 27 MB） | 手眼 RViz 插件的源码和构建环境；当前标定入口依赖其 install。 |
| `calibration_data/`（约 18 MB） | 标定会话、图像/点云和日志；记录有溯源价值。 |
| `.pytest_cache/`、各 `log/` | 可再生成的缓存与日志；日志可能包含尚需保留的诊断证据。 |

## 当前标定文件

- 左腕生效配置：`config/handeye/left/d405.json`；其历史候选与样本仍被文档或 ICP 工具引用。
- 右腕已选配置：`config/handeye/right/d405.json`；双臂启动读取它。来源样本与结果已保存在 `archive/handeye/right_d405/`。
- 中间 336L：最近保存结果在 `archive/handeye/fixed_336l/`；不要当作已接入生产配置。
- `small*`、`3*4*`、`Nero-7*`、`10.28*` 等样本和输出已归档；较早样本未推断相机身份。历史会话快照保留原路径，详见归档 README。

## 版本管理现状

工作树中有大量未提交和未跟踪文件，包含 `config/`、`calibration_ws/`、`hardware_ws/src/nero_dual_bringup/` 等实际在用内容。Git 子模块 `agx_arm_ros` 和 `agx_arm_sim` 也有本地修改。清理或重置前需要先决定如何保存这些工作；`git clean`、`git reset --hard` 会造成数据丢失风险。

## 建议顺序

1. 先确定保留范围：实机、仿真、历史标定、ICP 各自是否继续使用。
2. 清理确认不需要的缓存或日志；不要动仍被启动器使用的 install、虚拟环境和现场进程。
3. 将历史标定按相机/日期归档，逐项核对并更新路径引用。
4. 分别核对源码、配置、第三方依赖和结果数据的版本管理方式；不要把当前可用但未跟踪的配置当作废文件。
5. 验证常用启动入口、标定值与构建，再按需执行真实硬件验证。

# 固定中间相机与右臂持瓶：实物流程

左臂持孔板，中间 Gemini 336L 检测孔；右臂从侧面夹住瓶身，以瓶底对准孔，腕部相机位于瓶口一侧。此流程区分只读预览和实物控制：两段实物动作均需先在 RViz 检查、再输入 `do`。启动控制进程本身不执行轨迹。

## 前提与配置

- Ubuntu 22.04 / ROS 2 Humble，先按 [主 README](../README.md) 构建官方控制工作区；中间相机 Python 需有 `pyorbbecsdk`、OpenCV 和 NumPy。`ORBBEC_HOLE_PYTHON` 指向该 Python 解释器。
- 此现场记录为左臂 `can1`、右臂 `can0`，使用 `archive/handeye/fixed_336l/obzg2.launch.py`；换设备、移动相机或改变安装需重新标定及核验。深度到彩色的 SDK 外参不是手眼标定。
- `config/bottle_tool_preview.json` 的夹持点到瓶底 85 mm、法兰到夹持点及方向均为估计，瓶体和腕相机碰撞尺寸也为近似；孔板默认 400 mm 方形、8 mm 厚、板面内角度为估计。根据实物测量和 RViz 修改并核对。
- 孔板开孔由检测孔径决定（最大 120 mm）。检测偏大时，规划模型中的孔也偏大；人工检查孔边与实物空间。模型未覆盖全部支架及现场障碍。

## 一次性采集相机 SDK 外参

关闭占用相机的预览窗口，在仓库根目录运行；使用实际已安装 SDK 的 Python 替换路径：

```bash
export ORBBEC_HOLE_PYTHON=/path/to/orbbec-python
"$ORBBEC_HOLE_PYTHON" scripts/capture_orbbec_hole.py
```

将终端输出目录下的 `observation.json` 路径配置到**运行规划命令的终端**（每个新终端都需要设置）：

```bash
export ORBBEC_HOLE_EXTRINSIC=/absolute/path/to/orbbec_hole_observation_YYYYMMDD_HHMMSS/observation.json
```

程序校验相机序列号和深度内参；外参采集与预览必须使用匹配的相机及配置。采集文件是本机运行数据，不随仓库上传；没有设置变量时为历史本机路径，其他环境需显式设置。

## 实时检测和只读预览

```bash
cd /path/to/Nero-7
export ORBBEC_HOLE_PYTHON=/path/to/orbbec-python
bash scripts/preview_orbbec_hole.sh
```

检测画面绿椭圆需贴合孔边，按 `s` 保存当前帧，`q` 或 Esc 退出。另开终端启动只读规划 RViz（ROS domain 89）：

```bash
cd /path/to/Nero-7
bash scripts/preview_fixed_bottle.sh
```

在第三个终端设置 `ORBBEC_HOLE_EXTRINSIC`，再运行：

```bash
cd /path/to/Nero-7
export ORBBEC_HOLE_EXTRINSIC=/absolute/path/to/observation.json
bash scripts/preview_fixed_bottle_plan.sh
```

每次先重新按 `s`，保存帧须在 30 秒内。只读预览不发送运动；`--reuse` 仅供旧画面场景检查，不能用于实物执行。

## 实物分段执行

停止右臂重力补偿，确认现场连接、夹持与空间，在单独终端启动实物控制与 RViz（ROS domain 88；不要重复启动控制栈）：

```bash
cd /path/to/Nero-7
bash scripts/start_fixed_bottle_control.sh
```

让双臂和孔板静止，在相机窗口按 `s`。在另一个终端设置外参路径，并在 30 秒内运行：

```bash
cd /path/to/Nero-7
export ORBBEC_HOLE_EXTRINSIC=/absolute/path/to/observation.json
bash scripts/execute_fixed_bottle.sh
```

脚本更新孔板、瓶子和相机模型并规划到孔前，RViz 显示后终端提示输入 `do`；检查实物及轨迹后才执行。到孔前后再次确认瓶底对准、前送空间足够，检查前送轨迹并再次输入 `do`；其他输入取消对应段。默认孔前 100 mm、穿过板面 30 mm，不包含自动回退。前送速度缩放已调整，但新缩放尚未实物验证；控制器报告成功不等于已验证实际穿孔。

## 验证范围

2026-10-29 曾在现场以估计瓶子参数运行孔前及前送两段，控制器报告完成；未取得独立的穿孔成功测量。相机定位、瓶底 TCP、真实碰撞空间及前送速度仍需现场核验。临时帧、目标及历史轨迹不纳入仓库。

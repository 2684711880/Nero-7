# 中间固定 Gemini 336L：左臂持板眼在手外标定

相机固定，标定板由 left/can1 夹持，相机及机械臂基座在一次采集中保持不动。
板相对夹爪须固定，不可在采集中松动或重新夹持。结果为
`T_base_camera`：336L 彩色光学系到左臂局部 `base_link`，不是自动得到整机底盘坐标。
本任务不使用也不覆盖左腕 D405 的 `small2` 结果。

板参数：3 列×4 行，`DICT_5X5_250`，ID 0–11，黑色外框边长 60 mm，
相邻黑框净间距 6 mm。编码区域总尺寸 192×258 mm，不包含纸张白边。
用户照片中的板旋转了 180°；按码的方向与 ID 识别，无需手动重排 ID。
必须使用平整、刚性的背板，不可只夹住软纸后让板弯曲。

## 使用

先接好相机，单独验证图像、内参和识别（不打开 CAN）：

```bash
cd /home/a/Nero-7
bash scripts/run_orbbec_handeye.sh --camera-only
```

关闭测试窗口后，确认 `can1` 存在、反馈正常。由用户在独立终端启用左臂重力补偿：

```bash
cd /home/a/Nero-7
bash scripts/start_nero_gravity.sh left
```

该命令直接使能左臂并发送官方重力补偿控制。当前模型含标准夹爪，不含新板与支架负载；
需扶稳机械臂，不能假设松手后一定悬停。不要与其他左臂运动控制器同时运行。

另开终端启动采样：

```bash
cd /home/a/Nero-7
bash scripts/run_orbbec_handeye.sh --can can1
```

采集程序仅开相机、接收 CAN、发布 FK，不发送机械臂运动指令。
程序使用独立 ROS domain 78；若从其他终端查询其话题，同样设置此 domain。
有多台 Orbbec 时须追加 `--serial 实际序列号`。
SDK 环境使用现有 Orbbec venv；可用 `ORBBEC_HANDEYE_PYTHON` 指定另一个兼容 Python。

RViz 的 HandEyeCalibration 应为 Eye-to-Hand、sensor=`orbbec336l_color_optical`、
base=`base_link`、eef=`gripper_flange`，板尺寸 0.060/0.006 m。
左臂停稳至少 0.6 秒后 Take Sample，建议人为等待 1–2 秒。
采集 15–25 组不同位置和朝向，包含绕多个轴转动；不要只平移。
只在板清晰、检测稳定、机械臂反馈新鲜时采样。相机只输出校正后的彩色图像和匹配内参，
PnP 不使用深度距离。畸变模型不能直接支持时明确报错，不套用其他相机内参。

Solve 后单独保存 samples 与标定结果到终端打印的 session 目录，
保留额外未参与求解的姿态做独立验证。窗口关闭或终端 Ctrl+C 结束采集；
重力补偿由其独立终端控制，退出采集不会自动关闭补偿。

## 当前验证范围

已通过附件检测板 ID/排列、SDK/ROS 导入和离线几何检查。
准备时 USB 列表未发现 336L，系统也没有 can1，因此尚未验证实时图像、CAN/FK 或实机标定精度。

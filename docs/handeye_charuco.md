# NERO 末端 RealSense 手动手眼标定

## 当前：3×4 ArUco 多码板、D405、末端相机

默认配置为用户新上传图像的 `DICT_5X5_250`、ID 0～11，横向 3 列、纵向 4 行。
附件与 OpenCV GridBoard(3,4)、markerSize=200px、separation=20px、margin=20px 生成图逐像素一致。
当前缩小版单码黑框用户确认 30 mm，码间白色净间距 3 mm。
对应 RViz `measured marker size=0.030 m`、`measured separation=0.003 m`；
整板黑框外侧总宽 96 mm、总高 129 mm，不含外围白边。

用户自行连接 D405 与 CAN、启动重力补偿后：

```bash
cd /home/a/Nero-7
bash scripts/run_handeye.sh --can can1
```

多台 RealSense 同时连接时，D405 使用 `--serial 262622274345` 明确选择。
只看相机、不连接机器人：`bash scripts/run_handeye.sh --camera-only`。
Target 页参数：Aruco、X=3、Y=4、first marker ID=0、DICT_5X5_250、黑框0.030m、间距0.003m。
原有 SUBPIX 角点细化、OpenCV 多码 GridBoard 位姿估计、机器人 FK 和手眼求解流程不变。
多码共同估计一个板面位姿，不是分别解 12 份手眼外参。求解器手动选 OpenCV/Park1994。
识别框和坐标轴稳定后，改变末端位置与朝向，停稳后 Take Sample，采约20～30组。
板、基座及相机安装保持固定；退出重力补偿不属于采样准备步骤。
Solve 后将 Save samples / Save camera pose 保存到终端打印的本次目录，勿加载旧单码样本。
旧单码可用 `--target aruco`，旧 ChArUco 用 `--target charuco`；每次目标配置会随本次采集保存。
本次只切换采样板参数，不启动硬件、不自动导入结果、不修改现有指点流程。

## 历史：100mm 单码

此前单码配置为 `DICT_6X6_250`，ID `23`，
黑色外框边长用户实测 **100 mm**（不含白边）。照片已经 OpenCV 检测验证为 ID 23。
纸上 D435 字样是打印标题，不决定程序使用的相机型号。
相机可替换成 D435，使用相同入口和检测/求解流程；仅连接一台 RealSense 时自动选择，
多台同时连接则用 `--serial` 明确选择。每次从实际设备读取对应彩色模式的内参与畸变，
不复用 D405 内参。`inputs.json` 记录相机型号、序列号、分辨率及帧率。
换相机后新建样本重新标定，不能混用两台相机的样本或沿用原外参。

单独测试旧单码时运行 `bash scripts/run_handeye.sh --target aruco --camera-only`。
此模式不需要 `--can`，不创建 CAN socket、不读取机器人反馈或发布机器人 FK；
RViz 仍使用同一 ArUco 插件、100 mm/ID23 配置和 SUBPIX 检测，观察图像与坐标轴即可。
不进行手眼采样和求解。退出后相机采集进程随窗口关闭。

先自行启动重力补偿，再运行：

```bash
cd /home/a/Nero-7
bash scripts/run_handeye.sh --target aruco --can can1
```

Target 页应显示 Aruco、markers X/Y 各 1、first marker ID 23、DICT_6X6_250、
measured marker size 0.100 m。单码的 separation 参数仅满足插件接口，不影响单码几何。
每次生成独立目录并保存目标配置；不加载旧样本、不覆盖旧手眼结果。
固定纸张在平整硬板上；采集时保持板、机器人基座和相机安装不动。
手动换多个不同旋转轴的姿态，避免只平移或只绕同一轴；每次停稳后 Take Sample。
采集约 20～30 组后 Solve，保存 samples 和 camera pose 到本次目录。
换码本身不保证精度提高，求解后还需用未参与求解的姿态验证固定目标的一致性。
ArUco 插件已开启 SUBPIX 亚像素角点细化，替换原 CORNER_REFINE_NONE。
一次静止 D405 12 秒/86 帧回放对比：位置 RMS 0.162→0.105 mm，朝向 RMS 0.356→0.124°。
这只是一个姿态的短时重复性改善，不能解释或解决原有跨姿态约 17 mm 的标定不一致。
插件更改后须退出并重开标定 RViz 才生效；无需重启用户的重力补偿。
现有 `run_pointing.sh` 仍使用旧 ChArUco 目标和旧标定；新结果验证并接入前不要用它测试新码。

如要返回旧板：`bash scripts/run_handeye.sh --target charuco --can can1`。
以下旧板说明仅适用于 `--target charuco`；独立 `handeye.sh` 仍是 ChArUco 流程。

## RViz 手动采样（与之前的界面相同）

你自行启动重力补偿后，另开终端：

```bash
cd /home/a/Nero-7
bash scripts/run_handeye.sh --target charuco --can can1
```

打开 RViz HandEye Calibration 面板和实时图像。`can1` 是本次用户指定接口；
其他机械臂请明确指定接口。有多个 RealSense 时加 `--serial 序列号`。
板参数：7×5，方格 40 mm，编码黑框 30 mm，最长棋盘边 0.280 m。
插件使用 `DICT_5X5_250`，ID 0～16 与本板 `DICT_5X5_50` 编码相同。
若编码边长不是 30 mm，在 Target 页修改 `measured marker size (m)`。

Context 预设：Eye-in-hand；Sensor=`handeye_color_optical`；Object=`handeye_target`；
End-effector=`gripper_flange`；Robot base=`base_link`。

1. 板固定在相机视野内，图像出现识别框/坐标轴后，手动摆臂。
2. 停稳约 0.6 秒，在 **Calibrate** 页点 **Take Sample**。
3. 采集 15～25 个不同朝向，点 **Calibrate** 求解。
4. 分别保存样本与标定结果，建议存入终端打印的本次 `calibration_data/rviz_.../` 目录。

RViz 采集的是手眼变换样本；必须点击保存才能保留插件样本与结果。
下方独立 OpenCV 窗口入口另外提供自动保存照片/反馈，文件格式不同。
关闭本次 RViz 会关闭它的相机/被动反馈子进程，不停止用户的重力补偿进程。

D405 逆 Brown 畸变通过 RealSense SDK 反投影映射去畸变，发布匹配的零畸变 CameraInfo；
没有直接把逆 Brown 系数填成 OpenCV 正向系数。
只开启手眼标定需要的 RGB 流，从设备报告的 5～30fps 配置中优先选择高分辨率，再选择帧率。
D405 官方 RGB 规格为 1280×720@30（见下方官方链接）；当前 USB 2.1 连接下，
设备报告并实测能输出 1280×720@15 的独立彩色流。不能把 USB 2.x 等同于只能低分辨率。
启动时会打印实际选择的 USB 版本、分辨率和帧率。
CAN 过滤器已排除具有特殊语义的 CAN_ERR_FLAG。
实机短测 3 秒发布 84 帧校正图像、129 次静止反馈 FK；这是输入链路验证，不代表标定精度。

本工程自带插件源码与构建结果，无 ArmHub 运行目录依赖。重新构建：

```bash
bash scripts/build_handeye.sh
```

用户另行启动重力补偿、手动摆臂。本程序通过原始 SocketCAN 接收七轴反馈，
不调用 SDK connect/enable，不发送 CAN，不切模式，不执行运动。退出只关闭采集。

## 标定板与安装

- 使用本次照片对应的 **7×5** 格 ChArUco 板。照片是旋转 90° 的；不能只把配置交换成 5×7，编码位置会改变。
- 大方格实测 **40 mm**，对应 `square_m=0.04`。
- 照片的 17 个编码均可按 `DICT_5X5_50` 识别，ID 0～16；24 个棋盘角点全部检出。
- 编码标记黑色外边框边长从照片估计约 **30 mm**，尚未实测。量黑色外框，不含周围白边；采集时必须通过 `--marker-mm` 填实测值。
- 标定板固定在平整硬底板上，整次采集及独立验证期间保持不动；相机刚性固定在末端。
- 机械臂基座、升降、底盘均保持固定。变换输出基准是该单臂 URDF 基座，不是整机底盘坐标系。

## 启动采集

已验证本工程 `.venv-official` 中 NumPy 1.26.4、OpenCV-contrib 4.5.4、pyrealsense2 可导入。
无需 ROS 节点，不需要启动第二个硬件驱动。相机必须未被其他程序占用。
程序直接读取 RealSense 彩色流出厂内参，不估计新的相机内参。

先按原有流程自行启动需要的重力补偿。另开终端，下面假定你确认接口为 `can0`、标记实测为 30 mm：

```bash
cd /home/a/Nero-7
bash scripts/handeye.sh collect --can can0 --marker-mm 30
```

`can0` 仅为示例，请填所用机械臂的接口；左右臂分别采集与求解。
如接了多台 RealSense，必须加 `--serial 相机序列号`。
只开启彩色流，选择设备实际列出的最高分辨率及其可用帧率（5～30fps）。
此处不依赖深度图，也不要求彩色/深度使用共同配置。
官方产品规格：https://www.realsenseai.com/product-family/d405-series/
默认法兰为 URDF `gripper_flange`，不是夹爪尖端 TCP。

窗口按键：

- **空格 / c**：保存当前图像和末端反馈。停稳至少约 0.6 秒再拍；采集前后会检查七轴变化。
- **s**：用当前已保存的数据求解，并写 `result.json`。
- **q / Esc / Ctrl+C**：结束采集，已经保存的数据保留。

建议 15～25 组。不同方向转动腕部，同时覆盖合理的位置与距离；不要只平移或只绕单一轴转。
保持板在视野内、角点清晰。每次保存会显示角点数和重投影误差，默认拒绝 RMS >1.5 px 的帧。
运动检查阈值为 0.3°，不代表最终精度指标。没有硬件同步：通过主机接收时间配对，依靠停稳减少同步误差。

## 数据和离线求解

数据默认写入 `calibration_data/YYYYmmdd_HHMMSS/`，每次启动新目录，不覆盖旧数据。
可以通过 `--session /绝对路径/新目录` 指定新目录。

- `images/0000.png` 等：用于角点检测的彩色图像；逆 Brown 相机为校正图。
- `images/0000_raw.png` 等：逆 Brown 相机的未校正原始图，保留供复查。
- `session.json`：板参数、相机内参/畸变/序列号、CAN、URDF 路径与哈希、坐标系和 OpenCV 版本。
- `samples.json`：图像角点与 ID、七轴弧度、主机时间、相机时间域/帧号、`T_base_gripper`、`T_camera_board`。
- `result.json`：OpenCV PARK 求解结果、逆变换、训练数据中固定板的位姿一致性误差。

```bash
bash scripts/handeye.sh solve calibration_data/替换为采集目录名
```

约定 `T_A_B` 把 B 系坐标转换到 A 系。长度全部为米：

```text
T_base_board = T_base_gripper × T_gripper_camera × T_camera_board
p_base = T_base_gripper × T_gripper_camera × p_camera
```

输出 `T_gripper_camera` 即彩色相机光学坐标系到法兰坐标系的变换；
`T_camera_gripper` 为其逆。彩色光学坐标通常 X 向右、Y 向下、Z 向前，
不能直接替代深度光学系或相机外壳坐标系。深度点需先对齐到彩色光学系或正确使用相机外参。

机械臂位姿来自本工程 URDF 正运动学，输入为官方反馈的 joint1..7：
0x2A5/0x2A6/0x2A7/0x2A9，大端有符号 int32，单位 0.001°。
模型、机械零位和反馈本身的误差会进入最终标定。程序不使用控制器欧拉末端反馈，避免混用 TCP 与法兰定义。

## 独立验证

训练一致性小不等于实机精度已通过。保持板和基座不动，再开一次采集，收集若干新姿态，
退出后用原结果检查新数据（不重新拟合）：

```bash
bash scripts/handeye.sh validate calibration_data/原采集目录 calibration_data/新验证目录
```

新目录生成 `validation.json`，显示固定板位置 RMS / 最大误差（mm）和朝向最大误差（°）。
相机、板配置、CAN、URDF 和坐标系必须一致；结果是否足够取决于后续抓取的误差预算。
如果板或基座移动，此验证不成立。

## 离线检查

```bash
.venv-official/bin/python -m unittest discover -s scripts/tests -p test_handeye_charuco.py -v
bash scripts/handeye.sh --help
```

离线识别某张图片（不打开相机和 CAN）：

```bash
bash scripts/handeye.sh inspect /绝对路径/图片.jpg --output /绝对路径/识别预览.jpg
```

当前验证范围：附件图案识别、合成手眼几何/变换方向、协议解码、URDF FK、
逆 Brown 去畸变、D405 与 can1 实时输入和 RViz 界面启动。
尚未代替用户采样，最终手眼精度和抓取成功率均未验证。

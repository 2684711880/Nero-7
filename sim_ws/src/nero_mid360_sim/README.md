# NERO MID360 / Super-LIO 仿真

Gazebo 碰撞射线 → Livox CustomMsg（逐点仿真时间）和同位理想 IMU → 本机 Super-LIO。

安装预设在 `mobile_dual_nero/config/mid360_mounts.yaml`，均为 AIR `base_link` 坐标系下的估计位置，单位 m/rad。`chassis_up` 为底盘上方正装；`chest_down` 为胸前绕 x 翻转 π。位置和姿态均可用启动参数覆盖，重启生效。模型、TF 和扫描共用该变换；雷达/IMU 内部外参是理想同位单位变换，与机器人安装外参分开。

仿真按 Livox 官方参数采用水平 360°、垂直 -7°～52°、盲区 0.1 m、10 Hz 帧率。默认 128,000 次射线/秒以控制 CPU 开销，支持改到 200,000；不是厂商非重复扫描花纹、真实测距噪声、光学材质或 IMU 噪声的完整复现。每个物理更新时刻发一小束射线；同束点共享时间，物理步之间会随底盘和关节运动变化，不是给一张静态点云伪造逐点时间。射线从 0.1 m 盲半径开始，盲区内遮挡没有完整光学建模。仅碰撞几何参与射线，价签等纯视觉装饰不出点。

机身、躯干、机械臂回波保留；`/mid360/diagnostics` 给出每帧 rays/self_hits/environment_hits/returns，不作隐式自体滤除。IMU 的比力为 `R^T(a_world - gravity)`、角速度为 `R^T omega_world`，正装静止 z≈+9.81，倒装静止 z≈-9.81 m/s²。姿态字段标记未知；不会把真值姿态/里程计输入 Super-LIO。

Super-LIO 使用独立话题 `/mid360/lidar`、`/mid360/imu`，输出 `/lio/odom`、`/lio/path`、`/lio/cloud_world`。其 `world → imu` 为算法估计树；Gazebo/MoveIt 使用原 `odom → base_link → mid360_link`，两棵树没有用真值相连。当前 `/lio/odom` 是雷达内理想 IMU 位姿，不能当成底盘原点。不要把两种安装的原始轨迹坐标直接相减，应先对齐各自初始传感器坐标。

建图自动保存关闭，避免 Super-LIO 清理本机实测地图目录。地图应通过单独订阅导出到每轮实验目录。

视场依据：[Livox MID360 官方参数](https://www.livoxtech.com/cn/mid-360/specs)。

便利店顶板下表面位于 z=3.10 m，视觉隐藏但参与射线和 MoveIt 碰撞。前侧上部面板也参与射线，半透明外观不代表已模拟玻璃透射。入口仍开放。

## 编译和运行（本机）

```zsh
cd /home/a/Nero-7
bash -c 'source /home/a/ws_livox/setup.bash; cd sim_ws; colcon build --packages-select mobile_dual_nero nero_mid360_sim --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF'
bash scripts/run_store_mapping.sh mid360_mount:=chassis_up
# 直接重新运行，自动切换：
bash scripts/run_store_mapping.sh mid360_mount:=chest_down
# 覆盖位置/姿态，单位 m/rad，仍相对 AIR base_link：
bash scripts/run_store_mapping.sh mid360_mount:=chest_down \
  mid360_xyz:="0.18 0 0.78" mid360_rpy:="3.141592653589793 0 0"
```

终端未设置相关环境变量时，脚本采用 ROS_DOMAIN_ID=89、GAZEBO_MASTER_URI=http://127.0.0.1:11489；已有设置时沿用当前终端的域和地址。结束使用启动终端 Ctrl+C。雷达数据在模型生成 8 个仿真秒后开始，给落地和控制器启动留时间。默认仅打开 Gazebo 与建图 RViz；建图 launch 不含实体驱动或自动行驶。另一个终端加载相同环境、域 89 后，可用现有 `air_keyboard` 控制底盘。建图节点的 TF 单独发布到 `/lio/tf`，避免污染 MoveIt 的 TF 树。

最终默认 **128,000 次射线/秒**；32,000 的早期稀疏试验和过小匹配体素产生了明显漂移，保留在验证目录，不作为验收结果。现使用 0.5 m 匹配体素、0.25 m 点云降采样。参数 `mid360_points_per_second:=32000` 可降负载，但须重新验证精度。原始点云保留 0.1 m 之外的机身回波；算法另沿用 Super-LIO 的 2 m 近距剔除，`lio_blind:=0.1` 可用于研究近距自体回波污染，不代表仍能稳定建图。近距剔除不会让被机械臂遮挡的远处场景重新可见。

验证脚本和数据位于 `/home/a/Nero-7/scripts/`、`sim_ws/validation/mid360_superlio/`。独立终端示例：

```zsh
source /home/a/ws_livox/setup.zsh
source /home/a/Nero-7/sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=89
cd /home/a/Nero-7
# 会在仿真中慢速后退约 0.45 m；不要同时操作键盘：
python3 scripts/verify_store_mapping.py --mount chassis_up --move \
  --output sim_ws/validation/mid360_superlio/new_run
# 会检查 11 个插值状态，再把双肩缓慢向外摆至 1.0 rad 并恢复：
python3 scripts/probe_arm_occlusion.py --output sim_ws/validation/mid360_superlio/new_run
```

`map.pcd` 仅由 Super-LIO 发布的点云累积、5 cm 体素导出；真值仅用于位置误差和射线落点核查，不输入建图器、不用于校正输出。`pipeline_passed` 与 `mapping_accuracy_passed` 分别表示数据链路和该次短程误差检查，不能据此推广为整店覆盖、长时间稳定、实机精度或安装位置优劣结论。

## 天花板开关

```zsh
bash scripts/run_store_mapping.sh mid360_mount:=chassis_up ceiling:=true
# 直接重新运行，对比无顶场景：
bash scripts/run_store_mapping.sh mid360_mount:=chassis_up ceiling:=false
```

`ceiling` 默认 `true`，仅便利店环境使用。关闭时同时去掉 Gazebo 顶板碰撞和 MoveIt 顶板障碍物；开启时顶板视觉仍隐藏，实际射线会返回天花板回波。切换时重新执行脚本，会自动重启本工程的当前仿真。无顶验证时给 `verify_store_mapping.py` 加 `--no-ceiling`，以使几何核查与场景一致。


## 重复启动与停止

`run_store_mapping.sh` 现在会自动停止同一工程、同一 ROS 域和同一 Gazebo 地址上的上一套建图仿真，再用新参数启动。修改安装位置、朝向或天花板开关后，直接重新执行原命令即可，不必逐个关闭 Gazebo、RViz 和建图节点。退出中的旧节点会有一个短暂清理等待。每次重启重新初始化建图；已经导出的结果文件保留。

仅停止、不再启动：

```zsh
bash scripts/run_store_mapping.sh --stop
```

进程归属按启动入口、工程工作目录、ROS_DOMAIN_ID 和 GAZEBO_MASTER_URI 同时核对，只处理该 launch 及其子进程。其他工程占用同一 Gazebo 地址时会报告冲突并保留它。

## RViz 视角操作

默认工具为 `Move Camera`，视角类型为 `Orbit`、目标坐标系为 `world`。鼠标放在点云画面内：左键拖动旋转，中键拖动平移，滚轮缩放。切换过选择工具后，可点击工具栏 `Move Camera` 或按 `M` 回到视角控制。配置曾漏写 `Tools`，已补齐；通过独立 RViz 中的实际鼠标拖动验证。

## 在地图中显示机器人

建图 RViz 默认启用 `NERO robot (LIO pose)`。手动添加时选择 `Add → RobotModel`，`Description Source = Topic`、`Description Topic = /lio/robot_description`、`TF Prefix = lio_robot`；RViz 的 `Fixed Frame` 保持 `world`。

显示树为 `world → lio_robot/base_link → lio_robot/...`，使用 Super-LIO 的 IMU 位姿乘安装外参逆变换得到底盘位姿 `T_world_base = T_world_imu × inverse(T_base_mid360)`。理想仿真中雷达与 IMU 同位，所以正装、胸前倒装和 xyz/rpy 覆盖均使用相同逻辑。双臂和轮组显示由现有 `/joint_states` 驱动；前缀和独立 `/lio/tf` 保证显示树不与原 Gazebo/MoveIt TF 冲突。此桥接只用于显示，不向建图、底盘或机械臂发布控制数据。

胸前倒装实际运行中已核查 `world → lio_robot/mid360_link` 与 `/lio/odom` 闭合，记录见 `sim_ws/validation/mid360_superlio/robot_display_check.json`。

## 选择建图算法与近距过滤

```zsh
# FAST-LIO2，胸前倒装，剔除距雷达小于 0.3 m 的点：
bash scripts/run_store_mapping.sh algorithm:=fastlio2 \
  mid360_mount:=chest_down ceiling:=true lio_blind:=0.3
# Super-LIO，用相同阈值做对照：
bash scripts/run_store_mapping.sh algorithm:=superlio \
  mid360_mount:=chest_down ceiling:=true lio_blind:=0.3
```

`algorithm` 默认 `superlio`。不传 `lio_blind` 时，Super-LIO 用 2.0 m，FAST-LIO2 用 0.3 m；数值为相对雷达原点的三维距离，不是距底盘边缘的水平距离。阈值为 0 时不做这项算法近距剔除，但传感器仿真本身仍有 0.1 m 盲区。改变参数后直接重跑，入口会自动重启这一轮。公平对照需要显式指定相同阈值、安装姿态和运动路径；仅切换默认算法时两者近距阈值不同。

在 RViz 取消勾选 `LIO map`，勾选 `Raw MID360 (before LIO filter)`，可查看进入建图前的青色原始回波。它包含自体点；显示为传感器原始扫描，未做逐点运动畸变补偿。恢复 `LIO map` 可看算法处理后的输出。两种显示都跟随所选算法估计位姿，但点的过滤程度不同。

不能用近距过滤开关消除物理视场盲区。当前理想水平安装、平地、不计机身遮挡时：底盘正装高约 0.31 m，向下视角仅 7°，最近地面交点约 `0.31/tan(7°)=2.52 m`；胸前倒装高约 0.76 m，向下可达 52°，最近地面交点约 `0.76/tan(52°)=0.59 m`。这是以雷达投影点为圆心的地面几何边界，近处竖直墙/障碍物不受同一个地面边界限制，实际还会被车体、躯干、机械臂挡住。此前只做了双肩约 0.25 rad 的小幅试验，不能据此认定真实作业动作不会干扰建图。

FAST-LIO2 复用本机 `/home/a/mid-360/ws/install/fast_lio`；入口仅把该包加入搜索路径，继续使用 `/home/a/ws_livox` 的同一份 Livox 消息依赖，不启动教学工程的运动仿真。传感器 Livox/IMU 发布改为可靠 QoS，以兼容 FAST-LIO2 的可靠订阅，同时兼容 Super-LIO。

FAST-LIO2 内部输出原样保留在 `/fastlio/odom`、`/fastlio/cloud_world`、`/fastlio/path`。`fastlio_output_adapter` 用最初 100 个静止加速度计样本确定固定的重力对齐旋转，只旋转输出坐标以供共用 RViz/机器人显示，标准输出仍是 `/lio/odom`、`/lio/cloud_world`、`/lio/path`。适配不向 FAST-LIO2 提供真值、里程计或姿态修正。两套后端互斥启动。快照配置分别为 `config/fastlio_sim.yaml`、`config/superlio_sim.yaml`。

默认两处安装的整臂几何保守上界已计算，详见 `sim_ws/validation/lidar_arm_blind/README.md`：底盘方案覆盖 0～0.8 m 升降全行程时建议 `lio_blind:=2.40`（原 1.65 仅适用于升降量为 0）、胸前方案建议 `lio_blind:=1.30`，仅用于剔除当前模型内机械臂/夹爪/腕部相机的直接回波。它会同时损失近处环境，且无法消除遮挡，不应解释为完全没有机械臂干扰。默认阈值未自动改大。更细致的近距方案应使用当前关节 TF 和机器人几何做自体点过滤，再单独评估运动遮挡。

### 建图时手动开车并摆动双臂

在另一个交互终端运行（ROS 域须与建图终端一致，默认 89）：

```bash
cd /home/a/Nero-7
source /opt/ros/humble/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89}
ros2 run ranger_air_gazebo air_keyboard --ros-args -p speed:=0.1 -p turn:=0.2
```

W/S 前后，A/D 横移，Q/E 旋转；按 **T** 开启/关闭双臂摆动，默认关闭。T 不改变底盘当前速度，因此可原地摆动，也可边开车边摆动。双肩 joint1 从当前姿态平滑摆动 ±0.30 rad（约 ±17.2°），左右镜像关节采用同号角度，实现左臂向前、右臂向后，再交换方向。单轮 8 个仿真秒，其他关节保持启动摆动时的角度；轨迹采用 sin³，起止速度均为零。下垂姿态模型中的手端前后总行程约 0.35 m。每轮间有碰撞检查的短暂停顿。

**空格同时停车并停止摆臂**，停止摆臂时用当前反馈位置发保持指令；Ctrl-C 同样停止后退出。启动需要推进中的 /clock、近期 /joint_states、左右轨迹控制器和 MoveIt /check_state_validity。只允许从接近下垂的姿态启动，每轮检查 33 个完整双臂采样状态；此检查不是连续碰撞检测，也不保证底盘随后移动时的路径无障碍。摆臂期间不要同时用 MoveIt Execute 或其他程序控制双臂。

此功能仅供当前 Gazebo 整车仿真。改变 lio_blind 不影响原始 /mid360/points；要观察机械臂直接回波，请查看 Raw MID360，较大的 blind 会将这些点从建图输入中剔除。

摆臂支持在中途停止后再次按 T：joint1 允许当前角度在 ±0.40 rad 内，从实际反馈角度平滑接续，循环中心固定为 0，避免反复启停导致中心累积偏移。启动拒绝时会显示具体关节、当前角度和允许范围。

### 按机器人形状过滤自身回波

`self_filter:=true` 开启独立的输入过滤节点；`false`（默认）沿用原始输入。两套算法均支持：

```bash
bash scripts/run_store_mapping.sh mid360_mount:=chest_down self_filter:=true ceiling:=true
bash scripts/run_store_mapping.sh algorithm:=fastlio2 mid360_mount:=chassis_up self_filter:=true ceiling:=true
```

开启时，未显式指定的 `lio_blind` 自动设为 0.15 m；关闭时保留原算法默认值。显式 `lio_blind:=...` 始终优先。可以通过 `self_filter_padding:=0.025` 调整每个包围盒各方向的外扩余量，单位米，允许 0..0.10。

链路：`/mid360/lidar` → `body_self_filter` → `/mid360/lidar_filtered` → 所选 LIO。原始 `/mid360/points` 始终保留；过滤后可看 `/mid360/points_filtered`。RViz 勾选 `MID360 after body filter` 对比绿色过滤点云，勾选 `Self filter body boxes` 看橙色半透明包围盒；观察时可暂时关掉 `LIO map`。

实现从当前 compose URDF 的 39 个碰撞部件构造局部定向包围盒，包括底盘、轮子、机身、双臂、夹爪、相机、雷达、安装板和升降导轨。网格使用局部包围盒近似，圆柱使用外接盒，所以会比实体稍大，可能误删贴近机体的环境点；并非精确网格求交。模型跟随关节反馈，不使用 Gazebo 碰撞实体标签、地图定位或真值进行点分类。MID360 固定在底盘/胸前，因此只需机器人内部相对运动学，不依赖全局定位结果。

每个点保持原始 XYZ、offset_time、reflectivity、tag、line；帧 header、timebase、lidar_id、rsvd 不变，仅删除命中的点并更新 point_num。以实际关节消息时间插值，按最多 5 ms 时间桶计算包围盒姿态；这是时间离散近似，不是逐点精确求姿态。启动等待覆盖整帧的关节历史，缺反馈、超过 0.1 s 的反馈间隔或历史不足时丢弃该帧并警告，绝不绕过过滤把原始回波送入 LIO。

此过滤只去掉自身点，不补全被遮挡环境，不包含未建模的线缆或抓取物。需要检查过滤余量、外参和时间误差；目前支持的是本项目两个固定雷达预设，不是装在运动末端的通用节点。验证入口：`scripts/verify_body_self_filter.py --output ...`；它只检查点字段保留、近处点保留和 LIO 管线，不将输出数量比较当成分类准确率。


### 上身升降与雷达随动

上身新增 `upper_body_lift_joint` 竖直移动关节，行程 **0～0.80 m**，上限速度 0.05 m/s。`lift_height:=0.0` 设置启动升降量；例如启动在 0.30 m：

```bash
bash scripts/run_store_mapping.sh mid360_mount:=chest_down self_filter:=true ceiling:=true lift_height:=0.30
```

这是在现有底盘上添加的理想直线滑台和后导轨模型，不包含真实丝杆、伸缩机构、线缆或升降装置的硬件控制。当前后导轨最高约 1.135 m；双臂下垂、升降到 0.8 m 时上身最高约 1.737 m。雷达预设 xyz/rpy 定义为升降量为 0 时相对 AIR base_link 的位置。`chest_down` 改为上身的刚性子节点，高度为 **0.76 + 升降量**；`chassis_up` 仍固定于底盘，高度 0.31 m。雷达和理想 IMU 始终一起运动，内部外参没有变化。

重新启动 `air_keyboard`：**R 升高 5 cm、F 降低 5 cm、空格停止全部**。升降会先关闭双臂摆动、停车；如果底盘还在减速，请等停稳后再按 R/F。升降期间不接收行驶/摆臂指令，防止碰撞检查时的机器人姿态随后被键盘改变。

也可在加载相同 ROS 域和工作空间的终端指定绝对升降量（先停车、关闭 T 摆臂，不要和其他升降命令同时执行）：

```zsh
cd /home/a/Nero-7
source /opt/ros/humble/setup.zsh
source sim_ws/install/setup.zsh
export ROS_DOMAIN_ID=89
ros2 run ranger_air_gazebo set_lift_height --height 0.40
```

命令从实际关节反馈开始，检查 11 个全机器人路径采样状态，再发平滑轨迹给 `/upper_body_lift_controller`，不是修改 Gazebo 实体位置。每次键盘 5 cm 约 2 个仿真秒；0 到 0.4 m 约 16 秒。MoveIt 中新增 `upper_body_lift` 规划组及 `lowered`、`raised` 命名状态。

胸前安装的显示节点按 LIO 时间戳插值升降反馈，将动态的 sensor→base 外参用于机器人显示，不把上身上升误当成底盘上升。自身过滤的运动链包含升降关节和导轨，保持原始 Livox 逐点时间。缺失时刻对应的升降反馈时，机器人显示会等待/丢弃该条位姿，不回退到旧固定外参。

独立域 90、Gazebo 11490 的验证结果见 `sim_ws/validation/upper_body_lift/`：两种安装均实际执行 0→0.20→0 m，检查控制反馈、相对 TF 和过滤/LIO 连续输出。胸前安装该次测试的估计底盘高度回程后漂移约 3.3 cm；这些测试不代表完整升降、行驶、定位精度或实机稳定性验收。

### 头顶雷达与可调倾角

新增 `mid360_mount:=head_tilt`，默认安装原点（升降量为 0 时）为 AIR base_link 下 `(0.05, 0, 1.02)` m，位于躯干顶部上方；位置是仿真支架估计值。雷达刚性连接到上身，离地原点高度为 **1.02 + 升降量** m。

```bash
# 头顶安装，向机器人前方倾斜 45°
bash scripts/run_store_mapping.sh mid360_mount:=head_tilt mid360_tilt_deg:=45 self_filter:=true ceiling:=true
# 角度可改，例如 30°，并从抬高 0.30 m 的上身启动
bash scripts/run_store_mapping.sh mid360_mount:=head_tilt mid360_tilt_deg:=30 lift_height:=0.30 self_filter:=true ceiling:=true
```

`mid360_tilt_deg` 单位为度，仅作用于 head_tilt，范围 -90..90，默认 45；0 为正装竖直朝上，正值绕机器人 +Y 轴向前方 +X 倾斜，负值向后。它是安装倾角配置，不是运行中的电动云台关节；修改后重新启动仿真和 LIO 生效。显式完整 `mid360_rpy:="roll pitch yaw"` 覆盖倾角参数，单位仍为弧度。

Gazebo、估计机器人显示、自身过滤使用同一安装配置，头顶预设与胸前预设都会随上身升降，底盘预设不会。MID360 内理想 IMU 与雷达一同倾斜，内部外参保持单位变换；建图仍不接收 Gazebo 真值。倾角影响视场、遮挡和地面覆盖，应在对应角度下重新验证建图精度；两个旧安装的 blind 几何上界不能直接当作头顶安装的计算结果。

### 静止时估计高度抖动的修复

IMU 现以实际 200 Hz 采样间隔内传感器原点的世界速度差计算平均加速度，再生成 `R^T((v_now-v_previous)/dt - gravity)` 比力，保留真实底盘/升降/转动产生的加速度。之前直接抽取 Gazebo WorldLinearAccel 的单个物理步数值，接触求解的高频脉冲经抽样会产生明显错误积分。不是对 RViz 根 TF 做平滑或固定高度，也没有把 Gazebo 位姿传给 LIO。

头顶 45° 静止 20 秒对照，估计机器人高度峰峰值从约 15.8 cm 降至 4.1 mm；实际 Gazebo 高度峰峰值均约 0.04 mm。真实升降 20 cm 再返回仍正常。证据与边界见 `sim_ws/validation/imu_sampling_jitter/README.md`。已启动的 Gazebo 仍持有旧插件，需要重启仿真才加载修复；重启会重新开始本轮建图。

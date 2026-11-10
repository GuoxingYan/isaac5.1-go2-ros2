# Go2 · Isaac Sim 5.1 · Ubuntu 22.04

使用工作站现有 `conda activate isaacsim_510` 环境（Python 3.11 / Isaac Lab 2.3），无需 Windows、WSL2 或 DLL 补丁。

```bash
./run_go2.sh                         # GUI，真实相机与 RTX LiDAR
./run_go2.sh --headless --self-test   # 六方向运动、RGBD、点云验证及 HTML 报告
./run_go2.sh --policy flat            # 平地策略；默认 rough
./run_go2.sh num_envs=3               # 多机器人
./ros2/start_bridge.sh               # 独立 ROS2 Humble 发布进程
./ros2/start_bridge.sh --rviz-only   # 使用已有桥接进程打开 RViz
./ros2/run_probe.sh                  # 记录收到的真实 ROS2 消息
```

GUI 中点击仿真窗口使其获得焦点，W/S 前后，A/D 横移，Z/C 旋转，松键停止，SPACE 紧急停止，1–9 / F1–F12 选择机器人。支持组合键。

默认配置使用 `interiorgs`，直接加载 `data/` 下的 839873 Gaussian、碰撞和 occupancy 资产；其他程序化障碍物和仓库环境仍可通过 `env_name=...` 显式选择。未实际运行的场景不能视为已验收。

## 目录与文件说明

以下按当前工作区说明。源码和配置逐文件列出；报告中不同运行目录的同名产物用途相同，按文件名集中说明。`data/`、生成的雷达资产、`outputs/` 和 Python 缓存被 `.gitignore` 忽略，因此其他检出中可能不存在。报告和日志是对应运行时的历史证据，不代表当前代码已重新验收；运行产物、第三方依赖和缓存的数量也会变化。

### 顶层文件与目录

| 路径 | 作用 |
| --- | --- |
| `README.md` | 项目使用说明、目录索引和联调接口说明。 |
| `.gitignore` | 排除 Python 缓存、运行输出、本地场景数据和生成的 MID360 资产。 |
| `MIGRATION_4.5_to_5.1.md` | 历史 Windows 环境中 Isaac Sim 4.5 到 5.1 的迁移记录，不是当前 Ubuntu 安装步骤。 |
| `isaac_go2_ros2.py` | 仿真主入口：启动 Isaac Sim，加载配置、场景、Go2、策略、传感器、文件桥及可选自检。 |
| `run_go2.sh` | Ubuntu 启动入口：激活 Conda 环境、设置共享目录等环境变量并启动仿真。 |
| `run_go2.bat` | 历史 Windows 启动脚本，包含机器相关的 Isaac Sim 路径。 |
| `key_debug.txt` | 历史键盘事件调试记录。 |
| `assets/` | 本地仿真资产；`assets/sensors/` 放传感器资产，仓库场景也可由外部资产目录提供。 |
| `cfg/` | Hydra 仿真配置与传感器参数配置。 |
| `ckpts/`、`ckpts/unitree_go2/` | Go2 运动策略的预训练模型。 |
| `data/` | 本地 InteriorGS 场景的视觉、碰撞、占据和结构数据；不随 Git 跟踪。 |
| `env/` | 场景加载、地形生成、Gaussian RGB 和网格射线求交实现。 |
| `go2/` | Go2 环境、运动控制、传感器配置及场景巡航代码。 |
| `media/` | 项目演示动画。 |
| `ros2/` | 仿真侧文件交换、ROS2 发布/订阅和话题探测工具。 |
| `rviz/` | ROS2 可视化配置。 |
| `scripts/` | 验收、自检、报告构建和键盘事件辅助脚本。 |
| `reports/` | 历史验收结果、日志、原始传感器数据和离线报告；部分运行目录可能为空或按需生成。 |
| `outputs/` | 本机运行生成的 Hydra 日志、进程间共享文件、地图、缓存和调试数据，不随 Git 跟踪。 |
| `tools/` | 当前没有项目源码文件；可能残留 Python 缓存。 |
| `.git/` | Git 版本管理内部数据，不属于仿真输入。 |
| `.agents/`、`.aws/`、`.codex/` | 当前工作区中的本地隐藏目录，目前没有文件；不是仿真运行所需的源码。 |
| `__pycache__/` 及各子目录同名目录 | Python 自动生成的字节码缓存，不是项目源码。 |

### 仿真配置、模型与场景数据

| 文件 | 作用 |
| --- | --- |
| `cfg/sim.yaml` | 默认环境、机器人数量、仿真频率、巡航路径、传感器开关和仿真窗口设置。当前文件设置 `env_name: interiorgs`，具体以本地配置为准。 |
| `cfg/our_go2_sensors.json` | D435、MID360、IMU 的安装参数、采样规格、ROS 点云字段及模型限制。 |
| `ckpts/unitree_go2/flat_model_6800.pt` | 平地运动策略检查点，由 `--policy flat` 选择。 |
| `ckpts/unitree_go2/rough_model_7850.pt` | 粗糙地形运动策略检查点，由 `--policy rough` 选择。 |
| `assets/sensors/mid360.usda` | 根据传感器配置生成的本地 MID360 近似 RTX 传感器资产。 |
| `data/839873.usdz` | InteriorGS 场景视觉资产，包含用于 RGB 渲染的 Gaussian 数据。 |
| `data/839873_collision.usd` | 同一场景的碰撞网格，用于物理碰撞及深度/LiDAR 射线求交。 |
| `data/occupancy.png` | 场景二维占据图，用于检查安全出生点及生成巡航路径。 |
| `data/occupancy .json` | 占据图的分辨率、原点和边界元数据；文件名中 `.json` 前有空格，代码也兼容无空格的 `occupancy.json`。 |
| `data/labels .json` | 场景实例标签及对应三维包围盒；文件名中 `.json` 前有空格。 |
| `data/structure.json` | 场景房间等结构几何信息。 |
| `media/sim-demo1.gif` | 第一段仿真演示动画。 |
| `media/sim-demo2.gif` | 第二段仿真演示动画。 |

### 场景与机器人源码

| 文件 | 作用 |
| --- | --- |
| `env/sim_env.py` | 创建稀疏、中等、密集障碍物场景，加载仓库、医院、办公室等场景并添加语义标签。 |
| `env/terrain.py` | 生成离散障碍物高度场，并转换为地形网格。 |
| `env/terrain_cfg.py` | 定义离散障碍物地形生成参数。 |
| `env/interiorgs_scene.py` | 加载 InteriorGS 场景资源和占据图，检查安全出生点并生成场景/路径信息。 |
| `env/gaussian_rgb.py` | 读取 USDZ 内的 Gaussian 数据，通过 `gsplat` 渲染 RGB；几何深度与碰撞由网格提供。 |
| `env/mesh_lidar.py` | 从碰撞 USD 提取并缓存三角网格，使用 Warp 射线求交生成网格深度和 MID360 近似点云。 |
| `go2/go2_env.py` | 定义 Go2 的 Isaac Lab 场景、动作、观测、命令、传感器及环境配置。 |
| `go2/go2_ctrl.py` | 处理键盘和 ROS 速度命令、超时停车，并加载 flat/rough 运动策略。 |
| `go2/go2_ctrl_cfg.py` | flat/rough RSL-RL 策略网络、算法和检查点配置。 |
| `go2/go2_sensors.py` | 创建和读取 D435 相机、RTX LiDAR 等传感器及其数据。 |
| `go2/sensor_profile.py` | 加载传感器 JSON，生成 MID360 近似射线方向和 USDA 资产。 |
| `go2/path_cruise.py` | 沿场景坐标系航点生成 Go2 速度指令；收到 ROS 导航命令时让出控制。 |

### ROS2、可视化与验收脚本

| 文件 | 作用 |
| --- | --- |
| `ros2/go2_ros2_bridge.py` | 仿真进程侧文件桥：写入机器人状态及传感器数据，读取返回的速度命令。 |
| `ros2/go2_file_bridge_wsl2.py` | 独立 ROS2 进程侧文件桥：读取共享文件并发布消息/TF，接收速度命令写回共享目录；文件名保留历史 WSL2 命名。 |
| `ros2/livox_pointcloud.py` | 构造含 `x/y/z/intensity/tag/line/timestamp` 字段的 MID360 风格 `PointCloud2`。 |
| `ros2/start_bridge.sh` | 加载 ROS2 Humble 并启动文件桥，可选择同时启动 RViz 或仅启动 RViz。 |
| `ros2/start_wsl2_bridge.sh` | 旧启动方式的兼容别名，转发到 `start_bridge.sh`。 |
| `ros2/run_probe.sh` | 加载本机 ROS2 环境并启动话题探测程序。 |
| `ros2/verify_topics.py` | 订阅和记录 ROS2 消息，配合速度命令与传感器运输验收。 |
| `rviz/go2.rviz` | Go2 点云、图像、坐标系等 RViz 显示配置。 |
| `scripts/validate_runtime.py` | 仿真运行时运动、传感器及命令自检，收集证据并调用报告生成器。 |
| `scripts/validate_hardware.py` | 针对 D435、MID360 和 IMU 配置及采样结果的检查。 |
| `scripts/validate_scenes.sh` | 批量执行障碍物/仓库场景和 flat/rough 多机器人短时冒烟测试。 |
| `scripts/x11_keyboard.py` | X11 键盘事件注入辅助代码，用于 GUI 按键验收。 |
| `scripts/build_report.py` | 根据验收 JSON 和传感器证据构建自包含 HTML 报告。 |

### 验收报告与证据

`reports/` 的每个运行目录只包含当次实际生成的部分文件；相同文件名在不同目录中含义相同，并不表示每个目录都有全部文件。

| 目录/文件 | 作用 |
| --- | --- |
| `reports/VALIDATED.md` | Ubuntu 迁移阶段的验收摘要及适用范围。 |
| `reports/our-go2/VALIDATED.md` | D435/MID360/IMU 阶段验收摘要及限制。 |
| `reports/module_interface_report.html` | 仿真、建图定位和导航模块的离线接口分析报告。 |
| `reports/environment.json` | 验收环境信息记录。 |
| `reports/scene_results.json` | 场景及多机器人冒烟测试汇总。 |
| `reports/report_preview.png` | HTML 报告浏览器渲染预览截图。 |
| `reports/cmd-test/` | 速度命令相关测试的报告、结果与轨迹。 |
| `reports/gui-cmd/` | GUI 和速度命令测试的传感器及运动证据。 |
| `reports/gui-six-pass/` | GUI 六方向运动验收证据。 |
| `reports/latest/` | 某次最新验收的报告和证据；可能被后续运行覆盖。 |
| `reports/our-go2/` | 我方 Go2 传感器阶段的报告和原始证据。 |
| `reports/scenes/` | 各场景与多机器人冒烟测试的日志及结果；`obstacle-{sparse,medium,dense}/`、`warehouse/`、`warehouse-forklifts/`、`warehouse-shelves/`、`full-warehouse/` 分别对应场景，`multi-flat/`、`multi-rough/` 对应多机器人策略，`multi-sensors/` 对应多机器人传感器。 |
| `reports/algorithm-interop-current/`、`reports/our-go2-{imu-transport,smoke,transport}/`、`reports/rosbag/` | 本地联调、运输、冒烟或 rosbag 证据目录，可能为空或由运行生成；`reports/rosbag/isolated/` 下按运行编号存放隔离采集/解析结果。 |
| `report.html` | 对应运行目录的离线可视化验收报告。 |
| `results.json` | 对应运行的检查项、测量值和结果。 |
| `ros_topics.json` | ROS2 探测到的话题、消息及运输证据。 |
| `sensor_metadata.json` | 图像、点云等传感器的采样和参数元数据。 |
| `sensor_profile.json` | 验收时使用的传感器配置快照。 |
| `imu_transport_evidence.json` | IMU 消息运输与时间间隔的补充证据。 |
| `color_image.png` | 相机 RGB 图像。 |
| `depth_image.npy`、`depth_image.png` | 米制深度原始数组及可视化图。 |
| `point_cloud.npy`、`point_cloud.png` | LiDAR 点云原始数组及可视化图。 |
| `rgbd_point_cloud.npy`、`rgbd_point_cloud.png` | RGB-D 反投影点云原始数组及可视化图，不等同于 LiDAR。 |
| `semantic.npy`、`semantic.png` | 语义分割标签原始数组及可视化图。 |
| `imu.npy`、`imu.png` | IMU 采样原始数组及可视化图。 |
| `trajectory.png` | 测试中的机器人运动轨迹图。 |
| `simulation_view.png` | 仿真窗口画面截图。 |
| `scenes/*/smoke.json` | 对应场景或多机器人运行的短时冒烟结果。 |
| `reports/acceptance.log`、`acceptance_final.log`、`final_acceptance.log` | 不同批次的综合验收日志。 |
| `reports/browser_qa.log`、`browser_final_qa.log` | 报告浏览器检查日志。 |
| `reports/cmd-test.log`、`gui-cmd.log` | 对应命令测试和 GUI 命令测试日志。 |
| `reports/gui_validation.log`、`gui_validation2.log` | 不同批次 GUI 验证日志。 |
| `reports/multi_sensors.log` | 多机器人传感器测试日志。 |
| `reports/probe.log`、`probe_final.log`、`probe_verified.log` | 不同批次 ROS2 话题探测日志。 |
| `reports/ros2.log`、`ros2_final.log` | 不同批次 ROS2 桥接日志。 |
| `reports/report_build.log` | 报告生成日志。 |
| `reports/scene_sweep.log`、`reports/scenes/*.log` | 批量场景扫描日志及各场景/多机器人运行日志，`full-warehouse-retry.log` 为重试记录。 |
| `reports/selftest.log`、`startup.log`、`verified_run.log`、`reports/our-go2/validation2.log` | 自检、启动、最终验证运行及我方传感器阶段验证日志。 |

### 运行时生成文件

| 路径/文件名 | 作用 |
| --- | --- |
| `outputs/<日期>/<时间>/` | Hydra 为单次运行创建的目录；`isaac_go2_ros2.log` 是运行日志。 |
| `outputs/<日期>/<时间>/.hydra/` | 本次 Hydra 配置快照：`config.yaml` 为合成配置，`hydra.yaml` 为 Hydra 自身配置，`overrides.yaml` 为命令行覆盖项。 |
| `outputs/our-go2-bridge/`、`outputs/bridge/` 及其他 `*-bridge/`/联调目录 | 仿真和 ROS2 进程使用的共享文件目录；具体目录可由 `GO2_DATA_DIR` 指定。 |
| 共享目录中的 `data.json`、`cmd_vel.json` | 分别保存状态/传感器元数据和 ROS 返回的速度命令。 |
| 共享目录中的 `color_<序号>.png`、`depth_<序号>.npy`、`semantic_<序号>.npy` | 轮转保存的 RGB、深度和语义数据。 |
| 共享目录中的 `lidar_<序号>.npy`、`lidar_aux_<序号>.npz`、`rgbd_points_<序号>.npy` | 轮转保存的 LiDAR 点云、辅助字段及 RGB-D 点云。 |
| `outputs/mesh_cache/`、`outputs/interior_mesh.npz` | 本地碰撞网格的转换缓存；具体文件由运行生成。 |
| `outputs/fastlivo-*/`、`outputs/*-map/` | 本地建图实验输出；可能包含 `GlobalMap.pcd`、`Scans/`、位姿和时间记录，是否存在及含义以对应实验为准。 |
| `outputs/debug-history/` | 本地历史调试/报告副本。 |
| `outputs/usd_tools/` | 本机安装或解压的 USD 第三方工具及其依赖，不是项目源码。 |
| `outputs/report-http.log`、`outputs/ros_command_request.json` | 本地报告预览服务日志及 ROS 命令测试请求文件。 |

## 环境

- Conda 默认位置 `/data/anaconda3`；可用 `CONDA_BASE` 覆盖。
- 默认 GPU 物理与推理；可传 `--device cpu`。
- `GO2_DATA_DIR`：共享目录，默认 `outputs/bridge`。
- `ROS_DOMAIN_ID`：默认 42；不强制远端 discovery server。
- 系统 ROS2 在 `/opt/ros/humble` 时优先使用，否则使用现成 `/data/apps/ros2-humble`；ROS2 使用系统 Python 3.10，与仿真 Python 3.11 分进程。
- 报告默认 `reports/latest/report.html`，`--report-dir` 可更改目录。

## 数据真实性

RGB 为渲染彩色图；深度为 `distance_to_image_plane` 的 float32 米制数组，以 ROS `32FC1` 发布。语义来自真实 segmentation annotator，绝不使用 RGB 灰度或 Canny 冒充。RTX LiDAR 点云与 RGB-D 反投影点云分别保存。四槽轮转限制文件数量，元数据与 cmd_vel 原子替换。外部速度指令 0.5 秒超时停止。

自检通过真实键盘 callback 注入六方向指令，并测量机器人局部坐标位移和实际转角；这与操作系统按键投递是不同的验收项目。报告必须如实标记实际执行范围。

历史 Windows 迁移说明保留于 `MIGRATION_4.5_to_5.1.md`，不作为 Ubuntu 安装步骤。

## 验收与原始证据

`reports/latest/report.html` 是可离线打开的自包含报告，包含 RGB、米制深度、RTX LiDAR 点云、RGB-D 点云、真实语义标签及运动轨迹。原始数组保留在同目录 `.npy` 文件，动作和消息证据在 `results.json`、`ros_topics.json`。

```bash
./run_go2.sh --self-test --os-keyboard # GUI + X11 真实窗口键盘事件验收
./scripts/validate_scenes.sh          # 七场景 + 两策略多机器人冒烟测试
python scripts/build_report.py reports/latest  # 重新生成报告
```

运行 ROS 指令验收前，应先启动 bridge 和 probe。`--self-test` 最后会请求 probe 通过真实 ROS2 `/unitree_go2/cmd_vel` 发送 0.5 m/s，测量位移并检查超时停止；它会移动机器人，测试中请勿手动操控键盘。仅 callback 注入模式不代表 GUI 操作系统键盘验收。`GO2_NO_RENDER=1` 适用于同时关闭两种传感器的物理冒烟测试。

已完成的工作站验收覆盖：七种场景（完整仓库实际加载 3470 个 Mesh）、flat/rough 两策略三机器人、三台机器人各自 RGBD/LiDAR、组合键/编号切换/急停。GUI 六方向和 ROS2 端到端测量以最新 HTML/JSON 为准。场景测试为短时冒烟测试，不等于长时间稳定性或任意障碍物上的策略性能保证。默认 ROS2 图像、点云和里程计话题对应机器人 0；共享元数据包含全部机器人状态。

## 我方 Go2 硬件分支

分支 `codex/our-go2-d435-mid360` 与 main 的差别：main 为通用同视场相机和 `Example_Rotary`，没有独立 IMU；本分支使用 `cfg/our_go2_sensors.json` 描述 D435、MID360 及其内置 IMU。

- D435：RGB 640×360、69°水平视场；深度848×480、87°水平视场，0.2–10米，30Hz模拟时间调度。两路内参分别输出，不冒充已配准图像。50mm 双目基线为规格元数据，本模型没有模拟主动双目匹配、投射器或噪声。
- MID360：本地 RTX OmniLidar 资产，360°水平、−7°到52°垂直、0.1–70米范围、10Hz窗口、200000条计划射线/模拟秒。程序化角序列近似非重复覆盖，**不是 Livox 专有扫描轨迹、驱动/UDP协议或反射率性能复刻**；实际返回点数取决于场景求交。
- MID360 IMU：Isaac Lab 刚体物理IMU，200Hz，包含重力比力和角速度；body→LiDAR 使用导航链的 `[0.187,0,0.0803]`、Y轴13°，LiDAR→IMU 使用 FAST-LIVO2 的 `[-0.011,-0.02329,0.04412]`、R=I。ROS `/livox/imu` 使用真实 bag 的 identity 姿态消息约定，不把仿真真值当融合姿态。
- 安装位置：D435 仍为仿真占位值 `(0.4,0,0.2)`；MID360 已对齐导航 processor 的 body→LiDAR 外参，尚未宣称完成硬件标定。
- ROS新增 `/livox/lidar`（标准 PointCloud2，非Livox CustomMsg）、`/livox/imu`、`/unitree_go2/front_cam/depth_info`；原话题继续保留。彩色和深度使用不同optical frame。

```bash
./ros2/start_bridge.sh
GO2_REPORT_DIR=reports/our-go2 ./ros2/run_probe.sh
./run_go2.sh --self-test --os-keyboard
```

本分支共享目录默认为 `outputs/our-go2-bridge`，报告为 `reports/our-go2/report.html`。所有采样率使用模拟时间，不承诺GPU负载下的墙钟实时速度。

规格来源：[D435](https://www.realsenseai.com/products/stereo-depth-camera-d435/)、[D400 datasheet](https://dev.realsenseai.com/download/42003/)、[MID360规格](https://www.livoxtech.com/cn/mid-360/specs)、[Livox SDK2 IMU速率接口](https://github.com/Livox-SDK/Livox-SDK2/blob/master/include/livox_lidar_api.h)。

## 三模块联调接口说明（2026-11-09）

本节把仿真、建图与定位、导航看成一条完整的接口链，而不是三个彼此独立的程序。结论分为四种证据等级：

- **源码/配置确认**：来自本仓库、`/home/foxy_box` 中的启动脚本、节点源码和 YAML 配置。
- **真实数据确认**：来自真实 Go2 rosbag 的 `metadata.yaml`、运行日志或消息字段。
- **仿真验收确认**：来自 `reports/our-go2/results.json`、`ros_topics.json` 和图像证据。
- **尚未端到端确认**：尚未把当前仿真数据实际喂给 FAST-LIVO2、localizer 和 NeuPAN 并完成闭环，不能把它写成通过。

完整的带流程图、参数对照表、差异分级和图片证据的离线报告位于 [`reports/module_interface_report.html`](reports/module_interface_report.html)。

当前阶段的“仿真如何联动 FAST-LIVO2 建图”专项报告位于 [`reports/mapping_simulation_report.html`](reports/mapping_simulation_report.html)。本轮已实际启动仿真、ROS2 文件桥和只读 FAST-LIVO2/SC-PGO mapping，并保存输入/输出话题、算法日志、GlobalMap 与 SVG 可视化，结果见 [`reports/mapping-link-run-20261109-173837/`](reports/mapping-link-run-20261109-173837/)。本轮只验证 mapping 输入与局部输出，不启动 localizer 或 navigation processor。

### 当前建图专项实现状态

- 场景只使用 `data/839873.usdz`、`data/839873_collision.usd`、`data/occupancy.png` 和 `data/occupancy .json`；代码不读取 `source_frame_backup`，不做 recenter 或平移。
- 当前从 occupancy 安全区域生成 scene-frame 矩形巡航 waypoint；它不是全图覆盖路线，完整执行状态以实际运行记录为准。巡航器逐控制周期计算 `vx`、`vy`、`omega`，实际控制字段是 `linear.x=vx`、`linear.y=vy`、`angular.z=omega`，没有 `linear.z=vz`。
- 当前 `ScenePathCruise` 长边目标限幅为 `|vx|<=0.32 m/s`、`|vy|<=0.10 m/s`、`|omega|<=0.60 rad/s`；大航向误差时置零线速度并原地转向。它们是策略输入目标，不是机体实测地速。2026-11-10 实测在第一条长边出现姿态失稳，约 19.03 s 因 `body_up_z=0.647` 触发保护并停止；全路线巡航和全场景 mapping 均未验收。运行日志、地图、点云、bag、轨迹、视频和图片均为本地中间/结果数据，不纳入 Git 推送。

巡航控制闭环如下：

```text
scene-frame waypoints + Go2 当前真值 pose/yaw
                 ↓
        当前线段误差与期望航向
                 ↓
    机体速度目标 [vx, vy, omega]
                 ↓
48维 policy observation 中的 base_vel_cmd
                 ↓
       冻结 Flat actor 输出 12 维
           关节位置目标动作
                 ↓
            PhysX 更新状态
                 └──────────→ 下一控制周期
```

`ScenePathCruise` 在当前段上计算 waypoint 距离、路径偏差和期望航向；当航向误差大于 `0.65 rad` 时，先令 `vx=vy=0`，按比例生成限幅到 `±0.6 rad/s` 的 `omega`。否则由机体前向/侧向误差生成限幅速度目标。策略观测还包含机体线/角速度、投影重力、关节相对位置/速度和上一动作；InteriorGS 使用 Flat policy，不把 RGB-D、LiDAR 或 waypoint 直接送入 actor。actor 的 12 维关节目标经 Isaac Lab/PhysX 执行后，新的真值位姿反馈给下一周期。

本次异常发生在第一条长边而不是矩形角点；中止前 `vx≈0.32 m/s`、`vy≈0.10 m/s`，路径误差升至约 `0.26 m`，随后 roll 快速增大并触发倾倒保护。初始 yaw 与接入段方向基本一致且初始姿态接近直立，因此现有证据不支持把初始朝向或角点单独定为根因，但一次运行也不能排除它们。当前没有逐周期保存完整 policy observation/action，无法断定是侧向速度目标、策略响应、接触/碰撞还是其他动力学因素导致失稳。建议后续先补齐观测/动作/接触记录并做单变量短段对比；若确认命令切换过急，再评估斜率限制和角点减速，保留倾倒保护。
- LiDAR 直接对当前 collision USD 的三角形做求交；20,000 条计划射线/帧不是返回点承诺，传感器文件样本约 19,997–20,000 个有效点。真实 mapping bag 输入约 19,968–20,064 点；本轮 FAST-LIVO2 日志实测接收 19,998–20,000 点/帧，预处理输出 6,666 点，`last_curv` 约 83.31–83.32 ms。
- `/livox/lidar` 构造与真实 bag 相同的 `x/y/z/intensity/tag/line/timestamp` 字段布局（`point_step=26`）。本轮 DDS 实测 fields、header frame、width、point_step 与预期一致；FAST-LIVO2 日志实测逐点时间被转换为 `0–83.32 ms` 的扫描相对时间。算法读到 IMU 并完成初始化/重力对齐，但 580 帧日志均显示 `After UndistortPcl, IMUpose size=0`，因此本轮没有确认 IMU 轨迹实际用于扫描去畸变。现有证据不能单独确定根因；要检查扫描相对时间与 header/IMU 时间窗口是否重叠、仿真时钟是否一致、IMU 是否覆盖整帧、IMU 采样间隔/队列状态和该版本去畸变实现的条件。`mid360.yaml` 的 `imu_time_offset=0`、LiDAR→IMU 外参为 `[-0.011,-0.02329,0.04412]`/R=I 是配置事实，不足以证明时间同步或去畸变输入队列有效。
- TF 明确提供 `body -> livox_frame` 和 `body -> lidar_livox` 两个传感器别名，body-to-LiDAR 使用导航脚本的 `[0.187, 0, 0.0803]` 和 Y 轴 13°；FAST-LIVO2 的 LiDAR-to-IMU 外参仍由其 `mid360.yaml` 消费，不在点云中重复变换。
- 本阶段不把 RGB-D、仿真 odom/root state、定位 `map -> body` 或导航 `/cmd_vel2` 当作 FAST-LIVO2 建图输入。FAST-LIVO2/SC-PGO 已在独立目录运行并成功保存 8,461 点 `GlobalMap.pcd`、42 个关键帧 scans；`topics_motion` rosbag 的 SQLite 文件可复查到 `/aft_mapped_to_init`、`/cloud_registered_local` 各 71 条。另有 `topics_final_metadata.yaml` 记载各 79 条，但对应 `topics_final_0.db3` 不在本次证据目录，故不作为已验证录包计数。mapping 从机器人已到路线中段后才启动，后续局部运动约 1.2 m；相对位姿增量约 `(1.204, -0.143) m`。这是局部联动证据，不是从路线起点覆盖全场景的地图验收。

### 1. 总体架构与数据流

```text
                         仿真模块（本仓库）
       Isaac Sim 5.1 + Isaac Lab + Go2物理/RSL policy
          ┌──────────────┬──────────────┬──────────────┐
          │ D435 RGB-D  │ MID360点云   │ MID360 IMU   │
          │ 30 Hz       │ 10 Hz        │ 200 Hz       │
          └──────┬───────┴──────┬───────┴──────┬───────┘
                 │ 文件桥/ROS2   │             │
                 ├───────────────┼─────────────┤
                 │ /livox/lidar  │ /livox/imu  │ /unitree_go2/odom
                 │ PointCloud2   │ Imu         │ /tf
                 ▼               ▼             │
       ┌───────────────────────────────────────┘
       │ 建图模块：FAST-LIVO2 + SC-PGO
       │ 输入 /livox/lidar + /livox/imu
       │ 输出 /aft_mapped_to_init、/cloud_registered_local、地图文件
       └──────────────────────┬────────────────
                              │ GlobalMap.pcd + 实时局部点云/里程计
                              ▼
       ┌───────────────────────────────────────┐
       │ 定位模块：GlobalMap匹配 + 初始位姿     │
       │ 输入地图、实时点云/IMU、/initialpose     │
       │ 输出 /slam/lidar_pose（map坐标系）      │
       └──────────────────────┬────────────────
                              │
                              ▼
       /slam/lidar_pose ──┐
       /scan（点云投影） ──┼─> NeuPAN/track2 ─> /cmd_vel2 [vx, vy, omega]
       /clicked_path ─────┘                         │
                                                   ▼
                         需要适配为 /unitree_go2/cmd_vel
                                                   │
                                                   ▼
                                          仿真 Go2 运动闭环
```

这里有两个不能省略的环节：

1. **建图和定位不是直接读取仿真 ground truth。** `/unitree_go2/odom` 可以作为仿真真值对照，但正式链路必须由 FAST-LIVO2/localizer 生成 `/slam/lidar_pose`，否则等于绕过了定位模块。
2. **建图真机的运动来自摇杆/人工巡航，仿真需要可重复运动源。** 当前已用 occupancy 安全区域生成 scene-frame waypoint 巡航器；它只负责驱动 Go2 采集建图传感器数据，不发布或冒充导航模块的 `/clicked_path`。

### 2. 仿真模块：当前职责、输入和输出

仿真模块负责物理世界、Go2 运动、传感器生成和 ROS2 文件桥。`ros2/go2_ros2_bridge.py` 在 Isaac Sim 侧写入共享数据，`ros2/go2_file_bridge_wsl2.py` 在 ROS2 Python 进程中读取并发布消息；速度命令沿相反方向通过 `cmd_vel.json` 原子文件返回仿真控制器。

**仿真输入**

| 输入 | 类型/字段 | 用途 |
| --- | --- | --- |
| `/unitree_go2/cmd_vel` | `geometry_msgs/msg/Twist`；使用 `linear.x`、`linear.y`、`angular.z` | 当前仿真控制入口；桥接到 Go2 速度控制 |
| 键盘 W/S/A/D/Z/C | GUI callback | 人工验证运动；不适合作为可重复建图运动源 |
| `cfg/sim.yaml` | 物理频率、场景、传感器开关 | 选择 Isaac Sim 场景和传感器 |

**仿真输出**

| 话题 | 类型 | 当前参数/语义 |
| --- | --- | --- |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 10 Hz；字段为 `x/y/z/intensity/tag/line/timestamp`，`point_step=26`；frame=`livox_frame`；另提供 `body -> lidar_livox` alias TF |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 200 Hz；frame=`livox_frame`；ROS 姿态按真实 bag 的 identity/default covariance 约定填写，融合应使用角速度和比力 |
| `/unitree_go2/odom` | `nav_msgs/msg/Odometry` | frame=`odom`，child=`unitree_go2/base_link`；仿真真值/里程计对照，不是定位输出 |
| `/unitree_go2/pose` | `geometry_msgs/msg/PoseStamped` | 与仿真 odom 同源 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom -> unitree_go2/base_link` 动态 TF |
| `/unitree_go2/front_cam/color_image` | `sensor_msgs/msg/Image` | `rgb8`，640x360，30 Hz；frame=`unitree_go2/front_cam` |
| `/unitree_go2/front_cam/info` | `sensor_msgs/msg/CameraInfo` | RGB 内参由仿真相机实时发布 |
| `/unitree_go2/front_cam/depth_image` | `sensor_msgs/msg/Image` | `32FC1` 米，848x480，30 Hz；frame=`unitree_go2/d435_depth_optical` |
| `/unitree_go2/front_cam/depth_info` | `sensor_msgs/msg/CameraInfo` | 深度内参独立发布，不冒充 RGB-D 已配准 |
| `/unitree_go2/front_cam/semantic_segmentation_image` | `sensor_msgs/msg/Image` | `32SC1` 语义标签，848x480 |

**仿真传感器参数**

| 传感器 | 已配置值 | 模型限制 |
| --- | --- | --- |
| D435 RGB | 640x360，HFOV 69°，`fx=fy≈465.603`，`cx=320`，`cy=180`，30 Hz | 理想 pinhole + RTX 几何渲染；没有主动立体匹配、投射器、多径和硬件噪声 |
| D435 depth | 848x480，HFOV 87°，`fx=fy≈446.803`，`cx=424`，`cy=240`，0.2-10 m，30 Hz | 与 RGB 分辨率/内参不同；当前代码单独发布两套 `CameraInfo` |
| MID360 RTX近似 | 水平360°，垂直-7°到52°，0.1-70 m，10 Hz，200000 planned rays/s | 程序化非重复角序列，不是 Livox 专有扫描轨迹、UDP 驱动或反射率性能复刻；实际点数由场景求交决定 |
| MID360 IMU | 200 Hz；body 下位置由 body→LiDAR 安装位姿与 FAST-LIVO2 LiDAR→IMU 平移组合得到；旋转使用 `R=I` 的算法外参 | 理想无噪声物理 IMU；不复刻硬件噪声；ROS 姿态只匹配消息约定，不作为融合真值 |

历史通用传感器验收报告中的 **45/45** 只覆盖运动、D435、MID360 点云、200 Hz IMU、ROS2 运输和速度命令超时停车；它不替代本节的当前建图输入 smoke，也不表示三模块全链路通过。当前建图专项证据见 `reports/mapping-link-smoke-final/`。

### 3. 建图与定位模块：输入输出和核心思想

#### 3.1 建图：FAST-LIVO2 + SC-PGO

FAST-LIVO2 以 LiDAR-IMU 紧耦合里程计为前端，当前 `mid360.yaml` 配置为 lidar-only：`img_en=0`、`lidar_en=1`、`lidar_type=7`（标准 `PointCloud2`）、`scan_line=4`、`imu_en=true`、`use_wheel_odom=false`。SC-PGO 按关键帧做回环/全局优化，当前关键帧间隔为 0.5 m、角度间隔为 5°，再保存全局地图。

**建图输入**

| 输入 | 类型/频率 | 关键参数 |
| --- | --- | --- |
| `/livox/lidar` | 当前实测 `sensor_msgs/msg/PointCloud2`，约10 Hz | 字段为 `x/y/z/intensity/tag/line/timestamp`，真实首帧 `timestamp` 为 FLOAT64 纳秒且首点对齐 header stamp；配置 `lidar_type=7`、`scan_line=4`、`blind=0.1 m`、`point_filter_num=3`、表面滤波0.5 m。建图日志样本实测约 19,968–20,064 点输入、3,179–3,220 点预处理输出；不能将配置参数简单解释为精确 1/3 保留率 |
| `/livox/imu` | `sensor_msgs/msg/Imu`，约200 Hz | `imu_int_frame=30`；`acc_cov=0.5`、`gyr_cov=0.3`、bias covariance均0.0001；时间偏移0 |
| 相机（配置存在但当前关闭） | 配置 topic `/left_camera/image` | `img_en=0`，当前 Mid-360 主链不消费相机 |

**建图输出**

| 输出 | 语义 |
| --- | --- |
| `/aft_mapped_to_init` | 建图坐标系中的相对 LiDAR 里程计，约10 Hz；也被 localizer 配置作为 `odom_topic` |
| `/cloud_registered_local` | 当前注册后的局部点云；localizer 的 `cloud_topic` |
| `/Laser_map`、实时轨迹/点云 | RViz 和算法运行时可视化 |
| `/slam/points` | `pointcloud_transform` 将局部点云转到 `map` 后的点云 |
| `GlobalMap.pcd` | 保存给定位使用的先验地图 |
| `Scans/`、`opti_poses.txt`、`odom_poses.txt`、`gravity_alignment.txt` | 关键帧、优化轨迹、原始里程计和重力对齐结果 |

#### 3.2 定位：先验地图匹配

定位启动时加载建图产生的 `GlobalMap.pcd`，接收实时点云和 IMU，并通过 `/initialpose`（`geometry_msgs/msg/PoseWithCovarianceStamped`）完成 RViz 初始位姿。`localizer.yaml` 当前关键值为粗匹配分辨率0.25 m、精匹配分辨率0.1 m、粗/精最大迭代5/10、评分阈值0.2/0.1、`use_gicp=true`、VPS/GPS关闭、更新频率20 Hz。

定位配置当前设置 `cloud_topic=/cloud_registered_local`、`odom_topic=/aft_mapped_to_init`、`imu_odom_topic=/LIVO2/imu_propagate`、`map_frame=map`、`local_frame=lidar`、`update_hz=20`；VPS/GPS关闭，GICP开启。定位输出的核心契约是 `/slam/lidar_pose`，当前导航代码按 `geometry_msgs/msg/PoseStamped` 订阅，并要求 `header.frame_id=map`。`slam_pose_to_tf.py` 再把它转换为动态 `map -> body` TF；`/slam/points` 是实时点云在地图坐标下的显示/下游输入。`map` 是由建图那次运行定义的全局坐标，不等于仿真 `odom`，也不能直接假设和仿真世界原点重合。

### 4. 导航模块：输入输出和核心思想

导航链路把 3D LiDAR 变成二维障碍物观测，再由 NeuPAN/track2 根据定位位姿和人工/预设路径计算速度。启动脚本的实际顺序是：定位/建图相关节点准备好 -> `livox_lidar_processor` -> `pcd_publisher` -> `track2` -> `publish_path_interp`。路径发布故意晚于 planner 订阅，避免一次性路径消息丢失。

**导航前处理 `livox_lidar_processor`**

| 输入/输出 | 类型 | 参数/处理 |
| --- | --- | --- |
| 输入 `/livox/lidar` | 同时订阅 `livox_ros_driver2/msg/CustomMsg` 和 `sensor_msgs/msg/PointCloud2` | DDS 类型必须精确匹配，所以代码保留双订阅；PointCloud2 实际只读取 `x/y/z` |
| `/livox/points_raw` | `sensor_msgs/msg/PointCloud2` | 0.1 s 累积窗口后的原始点云 |
| `/livox/points_aligned` | `sensor_msgs/msg/PointCloud2` | 使用导航外参转换到 `body` frame |
| `/scan` | `sensor_msgs/msg/LaserScan` | `angle_min=-pi`、`angle_max=pi`、`angle_increment=0.004363323`（约0.25°，约1440束）、`range_min=0.1`、`range_max=100`、高度筛选[-0.01,0.7] m；扫描窗口0.1 s |

导航处理器使用 `p_body = R_y(13°) * p_lidar + [0.1870, 0, 0.0803]`，并把 aligned cloud 和 `/scan` 的 frame 写为 `body`。这组外参与仿真 `base_link -> unitree_go2/lidar` 的安装参数不是同一套数值，必须在适配层统一，不能只改 topic 名称。

**地图、路径和规划器**

| 节点 | 输入 | 输出/参数 |
| --- | --- | --- |
| `pcd_publisher.py` | `GlobalMap.pcd` | `/pcd_map`，`PointCloud2`，frame=`map`；导航验证用 voxel=0.05 m |
| `publish_path_interp.py` | 路径 JSON | `/clicked_path`，`nav_msgs/msg/Path`，默认 frame=`map`；按 `segment_length` 插值，默认发布10 Hz |
| `neupan_real_robot_track2.py` | `/slam/lidar_pose`、`/scan`、`/clicked_path` | `/cmd_vel2`（`geometry_msgs/msg/Twist`）和 `/opt_trajectory`、`/ref_trajectory`（`nav_msgs/msg/Path`） |
| track2 planner | 规划 YAML | `step_time=0.1 s`、`max_speed=[0.4,0.1,0.6]`（`vx,vy,omega`）、控制频率默认10 Hz、mecanum footprint约0.7x0.35 m |

`/cmd_vel2` 的 `linear.x`、`linear.y`、`angular.z` 对应 `[vx, vy, omega]`。当前仿真只订阅 `/unitree_go2/cmd_vel`，因此导航闭环还缺一个命令适配：`/cmd_vel2 -> /unitree_go2/cmd_vel`，并需要确认 frame、速度限幅、超时停车和 Go2 policy 对横向速度的实际响应。

**导航位姿外参的代码现状需要重点核查：**`neupan_real_robot_track2.py` 注释写明要把 LiDAR pose 换算到机器人/IMU frame，注释中的安装参数为约 `[0.187,0,0.083]`、绕Y 13°；但 `_get_robot_pose()` 实际将 `lidar_dx=0`、`lidar_dy=0`、`lidar_yaw_offset=0`，所以运行时没有做该补偿。另一个 `slam_pose_to_tf.py` 直接把 `/slam/lidar_pose` 的数值作为 `map -> body` TF 发布，也没有在该节点内应用 LiDAR-body 外参。若 `/slam/lidar_pose` 表示 LiDAR pose（定位配置 `local_frame=lidar`），这里存在坐标语义风险；应由负责的接口适配层明确一次且仅一次地完成转换，不能对只读导航代码进行隐式假设。

### 5. 导航与建图定位的输入参数汇总

#### 5.1 真实 bag 和 ROS2 消息

| 对象 | 建图/定位真实数据 | 导航真实数据 | 当前采用的基准 |
| --- | --- | --- | --- |
| LiDAR topic | `/livox/lidar` | Mid-360 bag同为`/livox/lidar`；硬件LiDAR bag为`/point_cloud`，启动脚本 remap 到`/livox/lidar` | topic名按建图定位 `/livox/lidar` |
| LiDAR类型 | 实际建图 bag 为 `PointCloud2`；文档旧写 `CustomMsg` | Mid-360 bag 为 `PointCloud2`；processor 兼容两种 | `PointCloud2`，因为真实 bag 和 `mid360.yaml lidar_type=7` 一致 |
| LiDAR字段 | 已确认 `x,y,z,intensity,tag,line,timestamp` | processor 只读取 `x,y,z`；仿真已按 bag 布局提供全部字段 | 结构按建图 bag 对齐；仿真 intensity/tag 为明确的零值模型，未复刻反射率 |
| LiDAR频率 | 建图 bag约10.002 Hz；4296帧/429.572 s | Mid-360运行记录约10.005 Hz | 10 Hz |
| IMU topic | `/livox/imu`，`sensor_msgs/msg/Imu`，约199.864 Hz | Mid-360同名同类型，约199.885 Hz；硬件 bag原名`/imu_data_ros_nei`并 remap | `/livox/imu`，200 Hz |
| 真实 frame | LiDAR和IMU bag样本均为 `livox_frame` | raw LiDAR/IMU 均为 `livox_frame`；桥提供 `body -> livox_frame` 与 `body -> lidar_livox` 静态 TF alias；导航处理器仍可输出 `body` 对齐点云/scan | raw frame 已按建图基准对齐，aligned 输出的 `body` 语义由导航处理器负责 |
| QoS | bag 元数据记录 LiDAR/IMU depth=256、可靠、volatile | publisher 代码配置 keep-last/depth 256/reliable/volatile；本轮 DDS introspection 实测 reliable/volatile，History Depth 为 UNKNOWN | reliability/durability 已在运行端点确认；depth 256 仅有 bag 元数据与代码配置证据，未由本轮 DDS introspection 确认，且未测量丢包计数 |

#### 5.2 相机参数

当前 Mid-360 的 `mid360.yaml` 设置 `img_en=0`，所以相机不属于当前 lidar-only 建图定位的阻塞输入。但 localizer launch 仍加载 `camera_pinhole.yaml`，若以后启用视觉，必须重新标定或适配：

| 参数 | 建图/定位配置 `camera_pinhole.yaml` | 仿真 D435 |
| --- | --- | --- |
| 原始/输出尺寸 | 1280x1024，`scale=0.5`，算法有效尺寸约640x512 | RGB 640x360；depth 848x480 |
| RGB/单路内参 | `fx=1300.9925`、`fy=1300.7330`、`cx=639.1689`、`cy=488.6036`；缩放后约 `[650.496,650.366,319.584,244.302]` | RGB `fx=fy≈465.603`、`cx=320`、`cy=180` |
| 畸变 | `d0=-0.076160`、`d1=0.123001`、`d2=-0.001130`、`d3=0.000251` | 当前仿真按理想 pinhole 发布，未模拟硬件畸变 |
| 频率/同步 | 配置未给出硬件频率；视觉模式还需检查图像与 LiDAR/IMU 时间同步 | 30 Hz模拟时间；RGB和depth分别发布，非已配准图像 |

### 6. 字段和坐标差异：当前以建图定位为准

| 差异 | 影响 | 当前处理/结论 |
| --- | --- | --- |
| 文档写 `CustomMsg`，实测 bag 和 `mid360.yaml` 为 `PointCloud2` | 若按文档只发布 CustomMsg，FAST-LIVO2 的 `lidar_type=7` 和真实链路会不匹配 | 以 `PointCloud2` 为当前基准；导航 processor 保留双类型兼容 |
| 真实点云有 `intensity/tag/line/timestamp`，旧仿真桥只发xyz | processor目前只读取xyz，但 FAST-LIVO2 的时间/扫描线预处理可能依赖完整布局 | 当前仿真已补齐字段、offset、`point_step=26` 和逐点纳秒 timestamp；intensity/tag 为零值模型，尚未复刻反射率 |
| 真实 frame=`livox_frame`；旧仿真 frame=`unitree_go2/lidar` / `unitree_go2/mid360_imu` | TF 查询和点云坐标语义可能失败或重复变换 | 当前 raw 消息统一为 `livox_frame`，并提供 `body -> livox_frame`、`body -> lidar_livox` alias；不在建图阶段发布 `map -> body` 真值 |
| mapping LiDAR-IMU 外参 `T=[-0.011,-0.02329,0.04412]`、R=I；旧仿真使用 identity 位置假设 | 影响 FAST-LIVO2 紧耦合和重力/姿态估计 | 当前 IMU 物理采样位置按 body→LiDAR 与该 LiDAR→IMU 平移组合；FAST-LIVO2 YAML 仍负责算法外参，仿真不重复变换点云 |
| 导航 body-LiDAR 外参 `[0.187,0,0.0803]`、绕Y 13°；旧仿真 LiDAR `[0.2,0,0.2]`、rpy全0 | `/scan`、地图点云、机器人位姿可能出现平移/俯仰误差 | 当前 MID360 传感器和 TF 已采用导航处理器的安装位姿；这只解决仿真输入几何，不修改导航算法 |
| IMU实测 orientation 常为 identity/covariance 0；旧仿真用 `orientation_covariance[0]=-1` | 下游若错误相信 orientation，会改变融合行为 | 当前 ROS 消息匹配 bag 的 identity/default covariance 约定；建图仍应使用角速度/加速度，不把 identity 当融合姿态 |
| `/slam/lidar_pose` 为 map 下 LiDAR位姿；仿真 `/unitree_go2/odom` 为 odom 下 base真值 | 直接 remap 会把不同坐标系当成同一坐标系 | 仿真真值仅作 oracle 对照；正式导航必须等待 localizer输出 |
| 导航最终命令 `/cmd_vel2`；仿真控制入口 `/unitree_go2/cmd_vel` | 规划器输出不会自动驱动仿真 | 桥已保留同字段 `/cmd_vel2` 订阅和限幅，但不属于当前建图验收；定位/导航闭环仍未验证 |
| track2 注释说换算 LiDAR pose 到机器人 pose，但实现将 dx/dy/yaw offset 均设为0；Pose-to-TF 节点又直接把 pose 当成 `map -> body` | 当 pose 实际代表 LiDAR frame 时，planner 位姿和 body TF 语义可能不一致 | 必须用真实位姿定义/标定验证，在只允许修改仿真的前提下由仿真接口层给出明确 frame 语义与一次性转换 |
| 旧仿真发布 depth=1 的 LiDAR publisher；真实 bag记录 depth=256 | 高负载或慢消费者时可能丢帧 | 当前两路 publisher 配置 depth=256，且本轮 mapping 已收到消息并产出结果；DDS History Depth 显示 UNKNOWN，实际队列深度与丢包数仍未从运行端点量化 |

### 7. 仿真符合性结论

| 检查项 | 当前结果 | 说明 |
| --- | --- | --- |
| `/livox/lidar` topic和消息大类 | **mapping 联动实测通过** | 标准 PointCloud2 被 FAST-LIVO2 实际接收；字段、point_step、raw frame 已在 DDS 样本核对，预处理输出 6,666 点 |
| `/livox/lidar` 10 Hz | **仿真时间通过；墙钟受 RTF 限制** | 仿真时间频率约10 Hz；当前工作站 RTF≈0.06-0.07，ROS 墙钟实测约0.99-1.04 Hz |
| `/livox/imu` 200 Hz | **仿真时间通过；墙钟受 RTF 限制** | 仿真时间约200 Hz；ROS 墙钟实测约14.7-15.4 Hz。raw frame 与 identity/default covariance 约定对齐，不复刻硬件噪声 |
| D435图像和内参 | **仿真自身已验收，和视觉配置不一致** | RGB/depth真实渲染且参数自洽；当前 `img_en=0`，不是 lidar-only 阻塞项 |
| FAST-LIVO2建图输入联动 | **局部端到端通过** | Foxy chroot FAST-LIVO2 实测处理仿真输入：约20k点→6,666点，LIO/SC-PGO发布输出并成功保存GlobalMap；证据见 `reports/mapping-link-run-20261109-173837/` |
| occupancy 全路线地图覆盖 | **尚未验收** | mapping 在机器人走到路线中段后启动，本轮只有约1.2 m后续运动；需从出生点先启动mapping再跑完整巡航 |
| localizer定位 | **尚未端到端验证** | 还没有加载仿真生成地图并验证`/initialpose -> /slam/lidar_pose` |
| `/scan` 导航输入 | **尚未验证** | 仿真没有运行`livox_lidar_processor`并确认字段、frame、频率和高度过滤结果 |
| NeuPAN导航 | **尚未验证** | 缺少真实`/slam/lidar_pose`、`/scan`和`/clicked_path`闭环 |
| `/cmd_vel2`驱动仿真 | **尚未验证** | 当前桥只订阅`/unitree_go2/cmd_vel`，尚无正式适配器 |
| 真实地图与仿真场景几何一致性 | **本阶段不适用** | 当前使用 `data/839873.usdz`、`data/839873_collision.usd` 和 occupancy 原位坐标；尚未将 FAST-LIVO2 地图结果与该场景做定位成功率比较 |

**最终判断：仿真 LiDAR/IMU 到 FAST-LIVO2 的接口与局部建图联动已通过；实时吞吐和 occupancy 全路线覆盖仍未通过验收，三模块全链路也未完成。** 下一阶段的最小顺序应为：

1. 优化 Isaac Sim 场景/渲染负载，提升 RTF，使墙钟传感器速率接近配置值；
2. 从 occupancy 路线出生点先启动 FAST-LIVO2，再执行完整 route，验证位姿增量、关键帧覆盖和地图几何；
3. 评估确认地图覆盖后再启动 localizer，验证初始位姿、`map -> body` TF 和 `/slam/lidar_pose`；
4. 建图输入稳定后，再单独验证导航 processor 的 `/scan` 和指定路径 `/clicked_path`，跟随模式不在短期目标；
5. 只有完成以上实测，才能把符合性矩阵中的“尚未验证”改为通过。

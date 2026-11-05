# Go2 · Isaac Sim 5.1 · Ubuntu 22.04

使用工作站现有 `conda activate isaacsim_510` 环境（Python 3.11 / Isaac Lab 2.3），无需 Windows、WSL2 或 DLL 补丁。

```bash
./run_go2.sh                         # GUI，真实相机与 RTX LiDAR
./run_go2.sh --headless --self-test   # 六方向运动、RGBD、点云验证及 HTML 报告
./run_go2.sh --policy flat            # 平地策略；默认 rough
./run_go2.sh num_envs=3               # 多机器人
./ros2/start_bridge.sh               # 独立 ROS2 Foxy chroot 发布进程
./ros2/start_bridge.sh --rviz-only   # 使用已有桥接进程打开 RViz
./ros2/run_probe.sh                  # Foxy chroot 记录收到的真实 ROS2 消息
```

GUI 中点击仿真窗口使其获得焦点，W/S 前后，A/D 横移，Z/C 旋转，松键停止，SPACE 紧急停止，1–9 / F1–F12 选择机器人。支持组合键。

默认使用本地程序化障碍物场景，避免仓库云资产网络依赖。仓库环境可通过 `env_name=full-warehouse` 等配置选择；本地资产目录用 `GO2_WAREHOUSE_DIR` 指定，否则尝试 Isaac 资产服务器。未实际运行的场景不能视为已验收。

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
python -m scripts.build_report reports/latest  # 重新生成报告
```

运行 ROS 指令验收前，应先启动 bridge 和 probe。`--self-test` 最后会请求 probe 通过真实 ROS2 `/unitree_go2/cmd_vel` 发送 0.5 m/s，测量位移并检查超时停止；它会移动机器人，测试中请勿手动操控键盘。仅 callback 注入模式不代表 GUI 操作系统键盘验收。`GO2_NO_RENDER=1` 适用于同时关闭两种传感器的物理冒烟测试。

已完成的工作站验收覆盖：七种场景（完整仓库实际加载 3470 个 Mesh）、flat/rough 两策略三机器人、三台机器人各自 RGBD/LiDAR、组合键/编号切换/急停。GUI 六方向和 ROS2 端到端测量以最新 HTML/JSON 为准。场景测试为短时冒烟测试，不等于长时间稳定性或任意障碍物上的策略性能保证。默认 ROS2 图像、点云和里程计话题对应机器人 0；共享元数据包含全部机器人状态。

## 我方 Go2 硬件分支

分支 `codex/our-go2-d435-mid360` 与 main 的差别：main 为通用同视场相机和 `Example_Rotary`，没有独立 IMU；本分支使用 `cfg/our_go2_sensors.json` 描述 D435、MID360 及其内置 IMU。

- D435：RGB 640×360、69°水平视场；深度848×480、87°水平视场，0.2–10米，30Hz模拟时间调度。两路内参分别输出，不冒充已配准图像。50mm 双目基线为规格元数据，本模型没有模拟主动双目匹配、投射器或噪声。
- MID360：本地 RTX OmniLidar 资产，360°水平、−7°到52°垂直、0.1–70米范围、10Hz窗口、200000条计划射线/模拟秒。程序化角序列近似非重复覆盖，**不是 Livox 专有扫描轨迹、驱动/UDP协议或反射率性能复刻**；实际返回点数取决于场景求交。
- MID360 IMU：Isaac Lab 刚体物理IMU，200Hz，包含重力比力和角速度，使用雷达安装位置，IMU到雷达暂假设单位外参。ROS `/livox/imu` 为 `sensor_msgs/Imu`，姿态不可用标记 `orientation_covariance[0]=-1`，不把仿真真值当硬件融合姿态。
- 安装位置按用户选择沿用：D435 `(0.4,0,0.2)`，MID360 `(0.2,0,0.2)`（相对base），尚未实机标定。
- ROS新增 `/livox/lidar`（标准 PointCloud2，非Livox CustomMsg）、`/livox/imu`、`/unitree_go2/front_cam/depth_info`；原话题继续保留。彩色和深度使用不同optical frame。

```bash
./ros2/start_bridge.sh
GO2_REPORT_DIR=reports/our-go2 ./ros2/run_probe.sh
./run_go2.sh --self-test --os-keyboard
```

本分支共享目录默认为 `outputs/our-go2-bridge`，报告为 `reports/our-go2/report.html`。所有采样率使用模拟时间，不承诺GPU负载下的墙钟实时速度。

规格来源：[D435](https://www.realsenseai.com/products/stereo-depth-camera-d435/)、[D400 datasheet](https://dev.realsenseai.com/download/42003/)、[MID360规格](https://www.livoxtech.com/cn/mid-360/specs)、[Livox SDK2 IMU速率接口](https://github.com/Livox-SDK/Livox-SDK2/blob/master/include/livox_lidar_api.h)。

## InteriorGS 双表示场景（第二阶段）

第一阶段已固定在标签 `our-go2-d435-mid360-v1-validated`（`dc524ee`）；第二阶段在 `codex/our-go2-interiorgs`，不改写第一阶段证据。

将原始数据放在 `data/`：`839873.usdz`、`839873_collision.usd`、`occupancy.png`、`occupancy.json`。使用已有环境安装附加依赖，不替换 torch：

```bash
conda activate isaacsim_510
python -m pip install --no-deps -r requirements-gaussian.txt
./prepare_interiorgs.sh
GO2_DATA_DIR="$PWD/outputs/interiorgs-bridge" ./ros2/start_bridge.sh
GO2_REPORT_DIR=reports/interiorgs GO2_PROBE_SECONDS=1800 ./ros2/run_probe.sh
./run_interiorgs.sh --spawn-seed 42 --self-test --os-keyboard
./run_interiorgs.sh                 # 每次随机安全出生；WASDZC控制
```

RGB直接读取USDZ中NuRec的1,445,429个高斯、SH3系数，由gsplat渲染，并在GUI独立窗口实时显示。未用外部PLY替换模型。深度和MID360使用原始碰撞mesh的Warp CUDA三角形射线求交；机器人接触由同一mesh的PhysX碰撞承载。深度使用D435深度内参，输出轴向Z米值，RGB与深度视场独立。该分支在此场景中不依赖原生RTX相机投影或RTX LiDAR返回；报告明确记录实际后端。

occupancy白色为可通行，额外叠加mesh障碍并保留0.8米中心净空。该样本的像素映射经mesh独立核对为X翻转、Y不翻转，不能盲用于其他样本。种子可复现；不传种子则随机采样。原始高斯张量、mesh及occupancy统一使用米制Z-up坐标。离线USD检查器隔离安装在outputs/usd_tools，不向Isaac运行进程注入该USD库。

限制：传感器外参未实机标定；高斯RGB未合成机器人mesh，mesh深度/LiDAR目前也不包含机器人自身遮挡；没有硬件噪声、反射率、专有扫描轨迹或扫描运动畸变模型。MID360每10Hz窗口20000条射线，速率按模拟时间计算。首次gsplat CUDA编译需数分钟；当前负载下仿真慢于实时。短时运动验收不代表任意房间/障碍物的长期自主导航能力。仅单机器人、flat策略；此场景禁用语义输出，避免不一致投影。

报告：`reports/interiorgs/report.html`（自包含RGB、深度、两类点云、IMU、六方向高斯视图、occupancy轨迹及独立几何误差）。原始数组、逐项结果和场景来源保存在同目录。

## ROS2 Foxy chroot

本项目的 ROS2 桥接和探针使用本机 Foxy chroot，不再回退到 Humble。启动脚本依次加载 `/opt/ros/foxy/setup.bash`、`/root/fastlivo_ws/install/setup.bash` 和 `/root/GO2_fastlivo2/install/setup.bash`。先把项目和共享输出目录 bind mount 到 chroot 内，并设置路径：

```bash
sudo mkdir -p /home/foxy_box/root/isaac5.1-go2-ros2
sudo mount --bind "$PWD" /home/foxy_box/root/isaac5.1-go2-ros2
export GO2_FOXY_PROJECT=/root/isaac5.1-go2-ros2
export GO2_FOXY_CHROOT=/home/foxy_box
./ros2/start_bridge.sh
GO2_REPORT_DIR=reports/foxy GO2_PROBE_SECONDS=120 ./ros2/run_probe.sh
```

`GO2_DATA_DIR` 也必须是 chroot 内可见的路径。脚本在 Foxy chroot 缺失时会直接报错，不会悄悄使用宿主 Humble；验证报告输出到 `reports/foxy/report.html`。

数据来源：[InteriorGS](https://huggingface.co/datasets/spatialverse/InteriorGS)、[SAGE-3D Collision Mesh](https://huggingface.co/datasets/spatialverse/SAGE-3D_Collision_Mesh)、[SAGE官方代码](https://github.com/Galery23/SAGE-3D_Official)、[NuRec/3DGRUT](https://github.com/nv-tlabs/3dgrut)。

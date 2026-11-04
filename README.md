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
python scripts/build_report.py reports/latest  # 重新生成报告
```

运行 ROS 指令验收前，应先启动 bridge 和 probe。`--self-test` 最后会请求 probe 通过真实 ROS2 `/unitree_go2/cmd_vel` 发送 0.5 m/s，测量位移并检查超时停止；它会移动机器人，测试中请勿手动操控键盘。仅 callback 注入模式不代表 GUI 操作系统键盘验收。`GO2_NO_RENDER=1` 适用于同时关闭两种传感器的物理冒烟测试。

已完成的工作站验收覆盖：七种场景（完整仓库实际加载 3470 个 Mesh）、flat/rough 两策略三机器人、三台机器人各自 RGBD/LiDAR、组合键/编号切换/急停。GUI 六方向和 ROS2 端到端测量以最新 HTML/JSON 为准。场景测试为短时冒烟测试，不等于长时间稳定性或任意障碍物上的策略性能保证。默认 ROS2 图像、点云和里程计话题对应机器人 0；共享元数据包含全部机器人状态。

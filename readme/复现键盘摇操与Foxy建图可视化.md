

本文记录从宿主机启动 Isaac Sim、在 ROS 2 Foxy chroot 发布 LiDAR/IMU、RViz 实时查看传感器并启动 FAST-LIVO2 建图的步骤。仿真和 Foxy 通过共享文件目录交换数据；发布桥、IMU 可视化节点、RViz 与 FAST-LIVO2 均运行在 Foxy chroot 和同一个 ROS domain 中。

## 环境与注意事项

- 宿主机运行 Isaac Sim；FAST-LIVO2、传感器发布桥和 RViz 运行于 `/home/foxy_box` 的 ROS 2 Foxy chroot。
- 本文使用 `ROS_DOMAIN_ID=117`、`ROS_LOCALHOST_ONLY=1`。所有 Foxy 终端必须一致。宿主机 ROS domain 与这套 Foxy 文件桥无关，因为桥从共享文件读取仿真数据。
- 本文选择独立目录 `outputs/foxy-keyboard-mapping`。仿真和 Foxy bridge 必须使用同一个目录；不要让两个 bridge 同时发布同一套传感器数据。
- 仿真传感器仿真时间目标为 LiDAR 10 Hz、IMU 200 Hz。`ros2 topic hz` 测量的是接收端观测频率，墙钟结果受仿真 RTF 和系统负载影响；IMU 显示约 50 Hz 不一定表示传感器配置为 50 Hz。
- 键盘控制目标速度幅度较大。先在开阔区域短按单个方向键，确认松键和空格停车；不要长按或快速连按来代替低速限幅。
- 不使用宽泛的 `pkill`/`killall`。清理残留节点前先核对进程命令行和归属。

## 终端 1：启动仿真(宿主机)

```bash
# 进入仓库目录。
cd /data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2
# 指定本轮仿真与 Foxy bridge 共用的独立数据目录；mkdir 不会清空已有文件。
export GO2_DATA_DIR="$PWD/outputs/foxy-keyboard-mapping"
mkdir -p "$GO2_DATA_DIR"
# 启动 GUI 仿真、关闭相机传感器，并使用指定 JSON 路径。
./run_go2.sh sensor.enable_camera=False --use-path-json
```

等待 Isaac Sim 窗口和 Go2 正常出现，并确认 `GO2_DATA_DIR/data.json` 持续更新。`--use-path-json` 让仿真采用 JSON 路径配置（只用到了初始的位置）；键盘摇操仍需在仿真窗口获得焦点后操作。

在另一个宿主机终端确认文件持续更新：

```bash
# 查看数据文件的更新时间、仿真帧号和传感器时间。
cd /data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2
python3 -c 'import json,os,time; p="outputs/foxy-keyboard-mapping/data.json"; d=json.load(open(p)); print("age_s=",round(time.time()-os.stat(p).st_mtime,2),"frame=",d.get("frame"),"sim_time=",d.get("simulation_time"),"lidar_time=",d.get("lidar_time"),"imu_samples=",len(d.get("imu_samples",[])))'
```

`age_s` 应接近 0，且 `frame`、`sim_time` 随运行推进。

## 终端 2：挂载源码和本轮数据(宿主机)

若挂载点已存在，先用 `findmnt` 确认它们分别指向本仓库和本轮数据目录，==🔵如果内部存在数据，需要看下是否是最新的数据==；==🟠若复用挂载点但本轮数据目录不同，==先确认没有进程正在使用该挂载，再有针对性地调整挂载来源；**确保 Foxy 中 `/mnt/go2-live/data/data.json` 对应本轮正在更新的宿主机文件。**

不要对来源不明的现有挂载盲目覆盖。

```bash
# 检查两个挂载点的来源和只读状态。
findmnt -T /home/foxy_box/mnt/go2-live/src
findmnt -T /home/foxy_box/mnt/go2-live/data
```
如果识别到还未挂载：

```bash
# 进入仓库目录，供下方 bind mount 使用绝对路径。
cd /data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2
# 指定本轮仿真与 Foxy bridge 共用的独立数据目录；mkdir 不会清空已有文件。
export GO2_DATA_DIR="$PWD/outputs/foxy-keyboard-mapping"

# 创建 chroot 内的源码和传感器数据挂载点。
sudo mkdir -p /home/foxy_box/mnt/go2-live/src /home/foxy_box/mnt/go2-live/data
# 首次运行时，将仓库绑定到 chroot；之后将该 bind mount 设为只读。
sudo mountpoint -q /home/foxy_box/mnt/go2-live/src || sudo mount --bind "$PWD" /home/foxy_box/mnt/go2-live/src
sudo mount -o remount,bind,ro /home/foxy_box/mnt/go2-live/src
# 首次运行时，将本轮活跃数据目录绑定到 chroot；Foxy 只读数据，不写入仿真输出。
sudo mountpoint -q /home/foxy_box/mnt/go2-live/data || sudo mount --bind "$GO2_DATA_DIR" /home/foxy_box/mnt/go2-live/data
sudo mount -o remount,bind,ro /home/foxy_box/mnt/go2-live/data
# 检查两个挂载点的来源和只读状态。
findmnt -T /home/foxy_box/mnt/go2-live/src
findmnt -T /home/foxy_box/mnt/go2-live/data
```

## 终端3：启动唯一的Foxy ros2 topic发布（ chroot ）

以下初始化要在每个新开的 Foxy 终端分别执行。直接 chroot 与 pipeline 中的运行环境保持一致：先进入 chroot，再 source ROS 和两个工作区。shell 环境变量不会自动传递到其他终端。

```bash
# 从宿主机进入 Foxy chroot。
sudo chroot /home/foxy_box /bin/bash
```

进入 chroot 后执行：

```bash
source /mnt/go2-live/src/readme/setup_foxy_keyboard_mapping.sh
```


**清理检查：确保只有一份 Foxy bridge**：在启动新的 bridge 前，在已初始化的 Foxy 终端检查：

```bash
# 列出 Foxy chroot 内文件桥进程；正常应只有一个，若尚未启动则为零个。
ps -eo pid,args | grep "[g]o2_file_bridge_wsl2.py"
# 检查 ROS 图中 LiDAR 和 IMU 的 publisher 数量及节点信息。
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/imu
# 删除之前的bridge
pgrep -af python3
alias killros='pkill -9 -f "kit"; pkill -9 -f "isaac-sim"; pkill -9 -f "livo2"; pkill -9 -f "rviz2"'
```

启动 bridge 前应确认没有旧 bridge；运行后两个话题都应只有一个 publisher，节点名为 `go2_file_bridge`。若发现多份进程，先用 `ps -p <PID> -o pid=,args=` 核实它确实属于本流程的 Foxy 文件桥，再在其原终端按 `Ctrl+C`，或仅对核实过的 PID 发送 `kill -INT <PID>`。不要停止仿真进程、FAST-LIVO2、RViz 或无关 Foxy 节点。

**==🟢从共享文件发布 /livox/lidar、/livox/imu 等 ROS 话题。==**

```bash
# 确认 Foxy Python 能导入 ROS 和点云构造工具。
python3 -c 'import rclpy; from livox_pointcloud import make_livox_pointcloud; print("Foxy imports OK")'
# 从共享文件发布 /livox/lidar、/livox/imu 等 ROS 话题。
python3 /mnt/go2-live/src/ros2/go2_file_bridge_wsl2.py
```

保持该终端运行，不要再启动第二份 bridge。

## 终端 4（可选）：启动 IMU 可视化转换节点（ chroot ）

完成通用初始化后运行：

```bash
sudo chroot /home/foxy_box /bin/bash
source /mnt/go2-live/src/readme/setup_foxy_keyboard_mapping.sh
# 订阅 /livox/imu，并把线加速度和角速度转成 RViz MarkerArray。
python3 /mnt/go2-live/src/ros2/livox_imu_rviz.py
```

保持节点运行。它发布 `/livox/imu_vectors`；RViz 用两种箭头分别显示线加速度和角速度。原始 IMU 仍由 `/livox/imu` 提供。

## 终端 4：启动 RViz（ chroot ）

完成通用初始化后设置 X11 并打开专用配置：

```bash
sudo chroot /home/foxy_box /bin/bash
source /mnt/go2-live/src/readme/setup_foxy_keyboard_mapping.sh

# 指定宿主机图形显示和 X11 授权文件。
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
# 打开 Foxy RViz 配置，固定坐标系为 livox_frame。
rviz2 -d /mnt/go2-live/src/rviz/foxy_livox.rviz
```

该配置显示 `/livox/lidar` 原始点云和 `/livox/imu_vectors` IMU 向量。若要观察 FAST-LIVO2 输出的 `/cloud_registered_local`，可在 RViz 的 Displays 面板新增 `PointCloud2` 显示并将 Topic 设为 `/cloud_registered_local`；建图启动后再确认该话题已有消息。

## 终端 5：启动建图（ chroot ）

完成通用初始化后，先检查 publisher 数量：

```bash
sudo chroot /home/foxy_box /bin/bash
source /mnt/go2-live/src/readme/setup_foxy_keyboard_mapping.sh

# 查看输入话题的 publisher 数量；两者都应为 1。
ros2 topic info /livox/lidar
ros2 topic info /livox/imu

# 分别测量接收频率：
# 统计 LiDAR 接收频率；持续运行，按 Ctrl+C 停止统计。
ros2 topic hz /livox/lidar
# 统计 IMU 接收频率；持续运行，按 Ctrl+C 停止统计。
ros2 topic hz /livox/imu
```


`ros2 topic hz` 会持续输出按接收时间计算的平均频率，不会替你设置频率。目标仿真时间频率为 LiDAR 10 Hz、IMU 200 Hz；实际墙钟测量可能因 RTF/负载而较低，不能把 50 Hz 当作固定目标。==🔴若 publisher 数不是 1、没有持续消息或时间戳回退，先停止并排查，不要启动建图。==

确认输入正常后，按 `Ctrl+C` 结束当前 `topic hz` 命令，再启动 FAST-LIVO2：

```bash

sudo chroot /home/foxy_box /bin/bash
source /mnt/go2-live/src/readme/setup_foxy_keyboard_mapping.sh


# 进入 FAST-LIVO2 工作区。
cd /root/GO2_fastlivo2
# 为本次运行创建独立地图目录，避免覆盖既有结果。
SAVE_DIR="/root/GO2_fastlivo2/maps_keyboard_$(date +%Y%m%d_%H%M%S)"
# 创建地图和关键帧点云目录。
mkdir -p "$SAVE_DIR/Scans"
# 打印并记下本轮地图保存位置。
printf 'This run saves to: %s\n' "$SAVE_DIR"
# 启动 FAST-LIVO2/SC-PGO，关闭其内置 RViz 并写入独立目录。
ros2 launch fast_livo mapping_mid360.launch.py \
  use_rviz:=False \
  avia_params_file:=/root/GO2_fastlivo2/install/fast_livo/share/fast_livo/config/mid360.yaml \
  save_dir:="$SAVE_DIR"
```

保持建图前台运行。先让 Go2 静止，检查初始化日志和输出话题：

```bash
# 列出 FAST-LIVO2 常见输出话题。
ros2 topic list | grep -E 'cloud_registered_local|aft_mapped_to_init|Laser_map'
# 查看当前帧点云是否有 publisher。
ros2 topic info /cloud_registered_local
# 持续测量当前帧点云输出频率；按 Ctrl+C 结束此监视，不会停止建图。
ros2 topic hz /cloud_registered_local
```

若出现 `imu loop back`、`IMUpose size=0` 等时间同步/去畸变异常，或输入 publisher 数大于 1，不要把有 topic 名称当作建图质量通过；先停止运动，检查是否启动了重复 bridge 和输入时间戳。

## 停止并保存地图

完成采集后先在 Isaac Sim 窗口按空格停车，保持仿真、bridge 和 mapping 继续运行数秒。在另一个已初始化、同为 domain 117 的 Foxy 终端显式保存地图：

**保存路径：/root/GO2_fastlivo2/maps_keyboard_$(date +%Y%m%d_%H%M%S)**
```bash
# 请求 SC-PGO 显式保存地图。
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

确认返回 `success=True` 后，在该终端使用建图时记录的实际路径检查结果：

```bash
# 将路径改为本轮 mapping 启动时打印的实际 SAVE_DIR。
SAVE_DIR="/root/GO2_fastlivo2/maps_keyboard_实际时间戳"
# 检查全局点云和关键帧配套文件。
test -s "$SAVE_DIR/GlobalMap.pcd" && echo 'GlobalMap.pcd: OK'
test -s "$SAVE_DIR/opti_poses.txt" && echo 'opti_poses.txt: OK'
test -s "$SAVE_DIR/odom_poses.txt" && echo 'odom_poses.txt: OK'
test -s "$SAVE_DIR/times.txt" && echo 'times.txt: OK'
# 检查 Scans 下是否生成 PCD 关键帧。
find "$SAVE_DIR/Scans" -maxdepth 1 -type f -name '*.pcd' -printf '%f\n' | sort | tail
```

检查保存响应和文件后，再回到 mapping 前台终端按 `Ctrl+C` 正常关闭节点。**不要把 Ctrl+C 当作唯一保存步骤。** 最后分别在 RViz、IMU 可视化节点和 bridge 前台终端按 `Ctrl+C`；仿真是否关闭由用户决定。


## 终端 6：启动定位（ chroot ）



```bash
sudo chroot /home/foxy_box /bin/bash
source /mnt/go2-live/src/readme/setup_foxy_keyboard_mapping.sh

# 1. 进入 chroot 后，先 source 环境
source /opt/ros/foxy/setup.bash
source /root/fastlivo_ws/install/setup.bash
source /root/GO2_fastlivo2/install/setup.bash

# 2. 准备定位地图（把之前建好的地图拷到定位目录 global_map_dir）
#  如手动设置第 3 步的 global_map_dir ,也可跳过此步
rm -rf /root/localization_map
mkdir -p /root/localization_map
# 这里把路径换成你实际建好的地图路径
cp /root/GO2_fastlivo2/maps_20261104_165019/GlobalMap.pcd /root/localization_map/
# 如果有重力对齐文件，一并拷过去（没有会提示忽略，不影响）
cp /root/GO2_fastlivo2/maps_20261104_165019/gravity_alignment.txt /root/localization_map/ 2>/dev/null || true

# 3. 启动定位节点（前台运行，会自带弹出一个 RViz，Ctrl+C 结束）
cd /root/GO2_fastlivo2
ros2 launch localizer localizer_launch.py \
  global_map_dir:=/root/localization_map

```
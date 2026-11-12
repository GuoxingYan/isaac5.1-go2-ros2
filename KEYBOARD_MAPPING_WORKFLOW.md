# 用键盘摇操仿真并运行 FAST-LIVO2 建图

本文说明如何在当前 839873 仿真场景中，用 Isaac Sim GUI 键盘手动控制 Go2，同时向 Foxy chroot 内的 FAST-LIVO2/SC-PGO 提供 LiDAR/IMU 数据，最后显式保存并检查地图。本文只涉及建图，不启动 localizer、track2 或导航控制器。

## 先读：当前键盘控制的速度风险

当前 GUI 键盘回调没有低速倍率：`W/S` 直接把策略速度目标设为 `vx=+1/-1`，`A/D` 设为 `vy=+1/-1`，`Z/C` 设为 `omega=+1/-1`。这些是送给冻结 Flat policy 的目标，不是实测机身速度；键盘路径也不会经过 ROS bridge 的 `vx ±0.4`、`vy ±0.1`、`omega ±0.6` 限幅。

因此，短按只会缩短目标持续时间，不会降低按住期间的目标幅度。此前自动巡航使用较小的前向/侧向目标仍出现过倾倒中止。**在增加并验证键盘低速倍率之前，不建议长按或直接执行长距离全图建图。** 若仅做初步检查，应在开阔位置逐轴短按，确认松键和空格停车有效；不要同时按前进和横移，也不要用快速连续按键来代替速度限幅。

## 链路与验收目标

```text
Isaac Sim Go2 + 839873 collision mesh
    ├── 人工键盘 -> Flat policy -> PhysX -> Go2 状态
    └── MID360 LiDAR / IMU -> GO2_DATA_DIR 共享文件
                                  │
                                  ▼
                         ROS2 文件桥（Humble）
                    /livox/lidar  /livox/imu
                                  │ DDS
                                  ▼
                   FAST-LIVO2 + SC-PGO（Foxy）
       /aft_mapped_to_init  /cloud_registered_local  /Laser_map
                                  │ /map_save
                                  ▼
            GlobalMap.pcd + Scans/ + 位姿/时间配套文件
```

- 输入地图算法的是 `/livox/lidar` 和 `/livox/imu`。当前 FAST-LIVO2 配置为 `img_en=0`，建图不依赖 RGB-D；本流程关闭仿真相机以减少负载，但保留 GUI、碰撞 mesh、LiDAR 和 IMU。
- `/livox/lidar` 为标准 `sensor_msgs/msg/PointCloud2`，frame=`livox_frame`，配置 10 Hz 仿真时间；`/livox/imu` 为 `sensor_msgs/msg/Imu`，frame=`livox_frame`，配置 200 Hz 仿真时间。
- 建图结果使用本次独立目录，不覆盖 `/root/localization_map`、`data/GlobalMap_5cm.pcd` 或既有地图。完整地图是否可供下游定位使用，还要检查同次导出的 `opti_poses.txt` 等配套文件。
- ROS 话题频率以仿真时钟产生；墙钟观测频率会随 RTF 下降。低墙钟频率表示仿真慢，不要为了追墙钟频率而在映射开始前移动机器人。

## 0. 准备

使用项目原目录和用户准备的数据：

```text
/data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2
/data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2/data/path_occupancy.json
```

确认没有正在运行的旧仿真、旧 bridge、旧 mapping 或导航控制器占用相同 ROS domain。不要用宽泛的 `pkill -9` 清理进程；在各自终端正常 `Ctrl+C` 结束旧进程，并确认 ROS 图中没有旧的命令发布者。

本流程示例统一使用 `ROS_DOMAIN_ID=116`、`ROS_LOCALHOST_ONLY=1`。如果该 domain 已被其他运行占用，应给本轮另选一个 domain，并在所有宿主机和 chroot 终端使用同一个值。

## 1. 终端 A：启动仿真

在宿主机终端执行：

```bash
# 切换到项目目录，确保后续相对路径指向本仓库。
cd /data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2
# 统一 ROS 2 DDS 域编号，所有宿主机/chroot 终端必须一致。
export ROS_DOMAIN_ID=116
# 限制 DDS 仅在本机通信。
export ROS_LOCALHOST_ONLY=1
# 指定仿真与文件桥共享的传感器数据目录。
export GO2_DATA_DIR="$PWD/outputs/manual-mapping-keyboard"
# 禁止启动器附加相机渲染开关，但仍保留 GUI 仿真窗口。
export GO2_NO_RENDER=1
# 创建本轮独立输出目录；已存在时不会清空其中内容。
mkdir -p "$GO2_DATA_DIR"

# 启动平地策略 GUI 仿真；关闭自动巡航，以 JSON 首个位姿作为出生参考，并禁用相机传感器。
./run_go2.sh --policy flat --no-cruise \
  --path-file "$PWD/data/path_occupancy.json" \
  sensor.enable_camera=False
```

说明：

- 不要加 `--headless`，否则代码不会注册 GUI 键盘回调。
- `--no-cruise` 禁用 occupancy waypoint 自动巡航。
- `--path-file` 仅用当前路径文件的首点和首点朝向作为出生位姿；它不会驱动机器人沿路径走。当前代码从首 pose 的 `yaw` 读取朝向；若 JSON 只有四元数而没有 `yaw`，当前首点身份四元数对应的默认 `yaw=0`。
- 当前 `path_occupancy.json` 的首点约为 `(1.405, -10.565) m`、yaw=`0`。即使加了 `--no-cruise`，启动代码仍会对 JSON 中整条路径做 occupancy 中心点 clearance 检查；最近离线抽样的路径中心最小 clearance 约 `0.25 m`，这不是 Go2 足迹碰撞验证，也不是建议摇操时贴着路径边缘行驶。
- `sensor.enable_camera=False` 不创建相机/Gaussian RGB 管线；`GO2_NO_RENDER=1` 让启动脚本不附加相机传感器渲染开关，但不是 `--headless`。碰撞场景和交互窗口仍由 Isaac Sim 加载。
- 仿真窗口启动并显示 Go2 后，先不要移动。确认终端打印 `Press ... WASDZC to control`，并等待 mesh、传感器和共享数据开始更新。

## 2. 终端 B：启动 ROS2 文件桥

在另一个宿主机终端执行，环境变量必须与终端 A 一致：

```bash
# 切换到项目目录。
cd /data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2
# 使用与仿真相同的 ROS 2 域编号。
export ROS_DOMAIN_ID=116
# 使用与仿真相同的本机通信限制。
export ROS_LOCALHOST_ONLY=1
# 指向与仿真进程完全相同的共享数据目录。
export GO2_DATA_DIR="$PWD/outputs/manual-mapping-keyboard"

# 加载 Humble 环境并启动文件桥，从共享文件发布 ROS 话题。
./ros2/start_bridge.sh
```

保持 bridge 运行。它从 `GO2_DATA_DIR` 读取仿真传感器数据并发布 ROS2 话题。建图结束前不要关闭它。

## 3. 终端 C：检查传感器话题

宿主机另开一个终端，设置相同 domain，并加载本机 ROS2 Humble：

```bash
# 让本终端使用与仿真、文件桥相同的 ROS 2 域编号。
export ROS_DOMAIN_ID=116
# 限制 DDS 仅在本机通信。
export ROS_LOCALHOST_ONLY=1
# 优先加载系统 Humble；否则加载本机安装的备用 Humble 环境。
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  source /data/apps/ros2-humble/activate.sh
fi

# 列出 LiDAR/IMU 话题及消息类型，确认它们出现在 ROS 图中。
ros2 topic list -t | grep -E '/livox/(lidar|imu)'
# 查看 LiDAR publisher、订阅者与 QoS 详情。
ros2 topic info -v /livox/lidar
# 查看 IMU publisher、订阅者与 QoS 详情。
ros2 topic info -v /livox/imu
# 收取一条 IMU 消息并检查 frame、时间戳和数据字段。
ros2 topic echo /livox/imu --once
# 收取一条点云消息的 frame_id。
ros2 topic echo /livox/lidar --once --field header.frame_id
# 收取一条点云消息的字段定义。
ros2 topic echo /livox/lidar --once --field fields
# 收取一条点云消息的每点字节数。
ros2 topic echo /livox/lidar --once --field point_step
```

预期：两个话题都有 publisher 和持续消息；LiDAR frame 是 `livox_frame`，`point_step=26`，字段含 `x/y/z/intensity/tag/line/timestamp`。`topic info -v` 用于检查 QoS。频率可分别测量，运行一段时间后按 `Ctrl+C` 停止 hz 监视：

```bash
# 统计 LiDAR 话题的墙钟接收频率；按 Ctrl+C 结束统计。
ros2 topic hz /livox/lidar
# 统计 IMU 话题的墙钟接收频率；按 Ctrl+C 结束统计。
ros2 topic hz /livox/imu
```

若要由 Humble 文件桥发布并在宿主机 RViz 中查看建图点云，先确认仿真和文件桥已有新数据，再运行：

```bash
# 设置与桥接进程一致的 ROS 域和本机通信选项。
export ROS_DOMAIN_ID=116
export ROS_LOCALHOST_ONLY=1
# 加载 Humble 环境后打开专用配置；它显示 /livox/lidar，并以 livox_frame 为固定坐标系。
source /data/apps/ros2-humble/activate.sh
rviz2 -d "$PWD/rviz/mapping_livox.rviz"
```

该配置持续显示 `/livox/lidar`。Humble RViz 没有通用 IMU 向量插件；另开一个同 domain 终端可持续查看 IMU 数值（按 Ctrl+C 停止）：

```bash
# 使用与 RViz、文件桥相同的 ROS 域和本机通信设置。
export ROS_DOMAIN_ID=116
export ROS_LOCALHOST_ONLY=1
# 持续打印 /livox/imu 消息，以检查时间戳、角速度和线加速度。
ros2 topic echo /livox/imu
```

### Foxy chroot 内发布并可视化（当前已验证）

如果要让 FAST-LIVO2 与 RViz 都在 Foxy chroot 内运行，可将本仓库和仿真共享目录只读 bind mount 到 chroot。以下示例使用 domain `117`，与宿主机现有 Humble domain 隔离；仿真已启动时不要重启它。当前活跃目录由本轮 `GO2_DATA_DIR` 决定，本次检查到正在更新的是 `outputs/manual-mapping-keyboard`。若仿真启动时指定了其他目录，应将下面命令中的宿主机目录替换为该实际目录。

宿主机终端执行一次挂载（源目录只读挂入，传感器共享数据也只读）：

```bash
# 创建 chroot 内用于访问项目源码和当前传感器数据的挂载点。
sudo mkdir -p /home/foxy_box/mnt/go2-live/src /home/foxy_box/mnt/go2-live/data
# 将仓库只读映射到 Foxy chroot。
sudo mount --bind "$PWD" /home/foxy_box/mnt/go2-live/src
sudo mount -o remount,bind,ro /home/foxy_box/mnt/go2-live/src
# 将当前仿真实际写入的数据目录只读映射到 Foxy chroot。
sudo mount --bind "$PWD/outputs/manual-mapping-keyboard" /home/foxy_box/mnt/go2-live/data
sudo mount -o remount,bind,ro /home/foxy_box/mnt/go2-live/data
```

打开三个 Foxy 终端。每个终端都使用相同的环境和 `ROS_DOMAIN_ID=117`：

```bash
# 进入 Foxy chroot；enter 脚本会挂载 /dev、/proc、/sys 和 X11 socket 并配置工作区。
bash /home/foxy_box/root/enter_foxy_sim_env.sh
# 在进入的 chroot shell 中隔离到本流程的 ROS domain。
export ROS_DOMAIN_ID=117
export ROS_LOCALHOST_ONLY=1
```

终端 1 启动文件桥，读取只读共享目录并发布两个 sensor_msgs 话题：

```bash
# 选择映射进 chroot 的仿真数据目录。
export GO2_DATA_DIR=/mnt/go2-live/data
# 让 Python 找到仓库中的 PointCloud2 构造模块。
export PYTHONPATH=/mnt/go2-live/src/ros2
# 启动 Foxy 文件桥；其命令订阅端不会在本流程中收到控制命令。
python3 /mnt/go2-live/src/ros2/go2_file_bridge_wsl2.py
```

终端 2 启动 IMU 箭头可视化转换节点；它订阅 `/livox/imu` 并发布 RViz MarkerArray `/livox/imu_vectors`：

```bash
# 运行加速度与角速度向量转换节点。
python3 /mnt/go2-live/src/ros2/livox_imu_rviz.py
```

终端 3 启动 Foxy RViz，配置同时显示 `/livox/lidar` 点云和 `/livox/imu_vectors` 两个 IMU 向量：

```bash
# 将 RViz 窗口连接到宿主机 X11 显示。
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
# 加载专用配置，固定坐标系为 livox_frame。
rviz2 -d /mnt/go2-live/src/rviz/foxy_livox.rviz
```

Foxy 终端里可用 `ros2 topic list -t` 和 `ros2 topic hz /livox/lidar`、`ros2 topic hz /livox/imu` 验证话题。此流程让传感器数据通过共享文件进入 Foxy，再由 Foxy DDS 发布；不依赖 Humble↔Foxy DDS discovery。仿真、文件桥和 RViz 都应保持运行。结束时分别在各前台终端按 `Ctrl+C`，不要用宽泛进程清理命令。

配置目标是仿真时间下 LiDAR 10 Hz、IMU 200 Hz。墙钟 hz 可能因 RTF 显著更低；应同时看消息时间戳持续递增和 mapping 节点是否收到数据。若看不到消息，先核对 A/B/C 的 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 和 A/B 的 `GO2_DATA_DIR`，不要先启动建图或移动机器人。

## 4. 进入 Foxy chroot

在宿主机新开一个终端执行部署脚本：

```bash
# 在宿主机挂载运行环境并进入 Foxy chroot。
bash /home/foxy_box/root/enter_foxy_sim_env.sh
```

脚本会挂载 chroot 运行环境并进入 Foxy shell。进入后确认 `ROS_DISTRO=foxy`，然后为本终端设置同一个 ROS domain：

```bash
# 设置与宿主机桥接一致的 ROS 域编号和本机通信选项。
export ROS_DOMAIN_ID=116
export ROS_LOCALHOST_ONLY=1
# 确认 Foxy 能发现 LiDAR 和 IMU 输入话题。
ros2 topic list | grep -E '/livox/(lidar|imu)'
```

如果 Foxy 看不到宿主机 bridge 发布的话题，先不要启动 mapping；检查各终端 domain、localhost 限制和网络 discovery 是否一致。

## 5. 启动 FAST-LIVO2 / SC-PGO，再开始摇操

在 Foxy shell 中为本轮创建独立目录。保留 shell 变量，后续用它检查输出：

```bash
# 进入 FAST-LIVO2 工作区。
cd /root/GO2_fastlivo2
# 生成本轮独立输出目录并保存在变量中供后续命令复用。
SAVE_DIR="/root/GO2_fastlivo2/maps_keyboard_$(date +%Y%m%d_%H%M%S)"
# 创建地图目录和关键帧点云子目录。
mkdir -p "$SAVE_DIR/Scans"
# 打印本轮输出路径，便于后续检查。
printf 'This run saves to: %s\n' "$SAVE_DIR"

# 使管道中的 launch 失败状态能够传递给 shell。
set -o pipefail
# 启动 FAST-LIVO2/SC-PGO，指定参数文件和独立保存目录，并将日志同时写入文件。
ros2 launch fast_livo mapping_mid360.launch.py \
  use_rviz:=False \
  avia_params_file:=/root/GO2_fastlivo2/install/fast_livo/share/fast_livo/config/mid360.yaml \
  save_dir:="$SAVE_DIR" 2>&1 | tee "$SAVE_DIR/mapping_launch.log"
```

保持 mapping launch 前台运行。它会启动 FAST-LIVO2、SC-PGO 和点云转换节点。**先让机器人静止**，在另一个 Foxy 终端确认 `/map_save` 服务和 `/aft_mapped_to_init`、`/cloud_registered_local` 等输出已出现，再开始移动。可以关注 mapping 终端中的 LiDAR 点数、`get imu`、里程计以及是否有初始化/时间同步错误。

另一个 Foxy 终端用于检查和最后保存地图：再从宿主机执行一次 `bash /home/foxy_box/root/enter_foxy_sim_env.sh` 进入第二个 chroot shell（该命令用于挂载并进入 Foxy 环境），并执行：

```bash
# 统一此检查终端的 ROS 域编号和本机通信选项。
export ROS_DOMAIN_ID=116
export ROS_LOCALHOST_ONLY=1
# 检查地图保存服务是否已注册。
ros2 service list -t | grep map_save
# 检查 FAST-LIVO2/SC-PGO 输出话题是否已经出现。
ros2 topic list | grep -E '/aft_mapped_to_init|/cloud_registered_local|/Laser_map'
```

只有在服务和建图输出就绪后才开始键盘运动。启动顺序必须是：**仿真与 bridge 稳定 → mapping 启动并确认接收输入/输出 → 人工摇操**。这样 mapping 能从机器人出生点附近开始积累轨迹，而不是中途才加入。

## 6. 在 Isaac Sim 窗口中键盘摇操

先把 Isaac Sim 窗口置前，点击主 3D viewport 空白处使窗口获得键盘焦点。单机器人场景下按 `1` 选择机器人 0；随后使用：

| 按键 | 当前策略目标 |
| --- | --- |
| `W` / `S` | 前进 / 后退：`vx=+1 / -1` |
| `A` / `D` | 机体左移 / 右移：`vy=+1 / -1` |
| `Z` / `C` | 左转 / 右转：`omega=+1 / -1` |
| 松开按键 | 对应轴目标回到 0 |
| `Space` | 清除已按键并将所有速度目标归零 |

这些是机体坐标系的策略速度目标；不是相机移动命令，也不是直接给关节下指令。`Space` 只是把目标速度归零，物理惯性和策略响应意味着 Go2 不一定瞬间停住，因此始终留出制动空间。手动采图时应围绕场景中可行走区域移动，避免贴障碍、急转、原地快速旋转或同时叠加多个方向。重点让雷达看到不同视角下的墙面、柱体和稳定几何特征；不要只在原地旋转，也不要在机器人摔倒或保护触发后继续假设地图正常。

### 视角改变后键盘失效的恢复

当前 `camera_follow=False`，鼠标调整 viewport 视角只改变观察相机，不应改变 Go2 机身位姿。键盘回调绑定在 Isaac Sim 主应用窗口；焦点转到终端、菜单、其他面板或窗口后，按键可能不再送到机器人控制回调。

1. 将 Isaac Sim 窗口置前，确认仿真仍处于 Play 状态，而不是暂停。
2. 点击主 3D viewport 的空白处，不要把焦点留在终端、菜单或其他输入控件。
3. 按 `1` 重新选择机器人；当前代码会清除已按键状态和旧 ROS 速度命令。
4. 按 `Space` 清零，再单独短按并松开 `W`，观察机器人是否响应。
5. 若仍无响应，确认启动命令没有 `--headless`，并确认没有 ROS teleop 或导航节点同时向 `/unitree_go2/cmd_vel`、`/cmd_vel2` 发命令。

如果只有视角变了而机身位置没变，通常是 viewport 相机被鼠标操作了；不要通过乱按运动键来“复位视角”。先用鼠标重新把 viewport 对准机器人，再按上面的步骤恢复焦点和停车。

## 7. 停止运动并保存本次地图

到达本轮建图终点后，在 Isaac Sim 中按 `Space`，确认 Go2 停住。继续让 mapping 接收静止传感器数据数秒，然后在第二个 Foxy 终端显式调用保存服务：

```bash
# 设置与建图节点一致的 ROS 域编号和本机通信选项。
export ROS_DOMAIN_ID=116
export ROS_LOCALHOST_ONLY=1
# 调用 Trigger 服务，请求显式保存当前地图。
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

等待明确响应：

```text
success=True
message='Map saved successfully, name:GlobalMap'
```

因为第一个 Foxy 终端正在前台运行 mapping launch，文件检查要在第二个 Foxy 终端执行。把启动 mapping 时打印的完整路径原样填入 `SAVE_DIR`：

```bash
# 将此占位路径替换为启动 mapping 时打印的本轮实际 SAVE_DIR。
SAVE_DIR="/root/GO2_fastlivo2/maps_keyboard_把实际时间戳填在这里"
# 查看本轮输出目录及文件大小。
ls -lh "$SAVE_DIR"
# 检查地图和配套位姿/时间文件存在且非空。
test -s "$SAVE_DIR/GlobalMap.pcd" && echo 'GlobalMap.pcd: OK'
test -s "$SAVE_DIR/opti_poses.txt" && echo 'opti_poses.txt: OK'
test -s "$SAVE_DIR/odom_poses.txt" && echo 'odom_poses.txt: OK'
test -s "$SAVE_DIR/times.txt" && echo 'times.txt: OK'
# 列出 Scans 下排序靠后的 PCD 文件，确认关键帧点云已写出。
find "$SAVE_DIR/Scans" -maxdepth 1 -type f -name '*.pcd' -printf '%f\n' | sort | tail
```

`GlobalMap.pcd`、`Scans/`、关键帧位姿/时间文件应来自同一次运行。若服务没有返回成功、关键文件缺失或 `Scans/` 为空，不要把结果报告成完整建图成功，也不要用其他运行的文件补齐。

**确认 `/map_save` 成功并检查文件后**，才回到第一个 Foxy 终端按一次 `Ctrl+C` 正常关闭节点。不要把“Ctrl+C 时可能自动保存”当作唯一保存步骤。然后停止 ROS2 bridge，最后关闭 Isaac Sim。

已有运行中，显式 `/map_save` 成功后，Ctrl+C 退出阶段曾出现 FAST-LIVO2 自身的 `PCDWriter::writeBinary Error during open`，而 SC-PGO 同时记录 `Map already saved.`。两者是不同节点的保存路径；若再次出现，应以本轮 `/map_save` 的成功响应和 Ctrl+C 前已核对的 SC-PGO 地图/关键帧文件为准，并保留退出日志用于排查 FAST-LIVO2 的独立 PCD 写盘配置。

## 8. 结果位置与完成判据

- 地图目录在 chroot 内为 `$SAVE_DIR`，例如 `/root/GO2_fastlivo2/maps_keyboard_20261111_120000`。
- 宿主机可从 `/home/foxy_box/root/GO2_fastlivo2/` 下找到对应 `maps_keyboard_*` 目录；其中也保存 `mapping_launch.log`。
- `GlobalMap.pcd` 存在只表示点云地图文件已写出，不代表完整覆盖了所有目标区域、定位一定可用或机器人全程安全。需要结合 `Scans/`、`opti_poses.txt`、建图轨迹和运行日志判断质量。
- 地图采集范围不足时，应在 mapping 仍运行且机器人静止可控的前提下继续采集，再重新调用 `/map_save`。每次保存都检查 service response 和文件，不要覆盖其他实验目录。

## 常见故障速查

| 现象 | 优先检查 |
| --- | --- |
| GUI 看得到但按键无效 | Isaac Sim 窗口焦点、viewport 焦点、Play 状态、是否误用了 `--headless`；按 `1` 后 `Space` 清零 |
| 键盘动了但 LiDAR/IMU 没话题 | bridge 是否运行；A/B/C 的 ROS domain 是否一致；仿真 `GO2_DATA_DIR` 与 bridge 是否完全相同 |
| Foxy 看不到传感器话题 | 检查 chroot bind mount、`GO2_DATA_DIR` 是否指向正在更新的目录、Foxy 节点是否使用同一 `ROS_DOMAIN_ID`；本节 Foxy 文件桥方案不依赖跨发行版 DDS discovery |
| FAST-LIVO2 无有效输出 | 确认 `/livox/lidar` 为 PointCloud2、frame/fields 正确；检查 IMU 数据与时间戳、FAST-LIVO2 初始化日志；机器人先静止启动 |
| `/map_save` 服务不可用 | mapping launch 是否仍运行、domain 是否一致、`sc_pgo_node` 是否启动成功 |
| map 文件有但下游提示元数据缺失 | 核对本轮 `opti_poses.txt`、`Scans/` 和其他配套输出；只使用同一次建图产物 |


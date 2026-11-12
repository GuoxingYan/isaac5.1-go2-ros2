# FAST-LIVO2 (ROS 2 Foxy) 完整部署与运行手册
### update on 1105 by wangnana

## 环境说明
> **- 宿主机**：Ubuntu 22.04（不改动）  
> **- chroot 隔离环境**：Ubuntu 20.04 + ROS 2 Foxy，路径 `/home/foxy_box/`  
> **- chroot 内项目**：`/root/GO2_fastlivo2/`；bag：`/root/rosbag_Dataset/`  
> CPU 需支持 AVX2

### 每次电脑重启后需执行一次（宿主机）
```bash
sudo mount --rbind /dev   /home/foxy_box/dev   2>/dev/null
sudo mount --rbind /proc  /home/foxy_box/proc  2>/dev/null
sudo mount --rbind /sys   /home/foxy_box/sys   2>/dev/null
sudo mkdir -p /home/foxy_box/tmp/.X11-unix
sudo mount --bind /tmp/.X11-unix /home/foxy_box/tmp/.X11-unix 2>/dev/null
sudo cp /etc/resolv.conf /home/foxy_box/etc/resolv.conf
xhost +local:root
```

## 0: 进入foxy环境
```bash
sudo chroot /home/foxy_box /bin/bash 
source /opt/ros/foxy/setup.bash
source /root/fastlivo_ws/install/setup.bash
source /root/GO2_fastlivo2/install/setup.bash

# 统一本流程 Foxy 节点使用的 ROS domain 和本机通信选项。
export ROS_DOMAIN_ID=117
export ROS_LOCALHOST_ONLY=1

```
验证:
```bash
echo $ROS_DISTRO
ros2 topic list
```
以下内容全部默认在foxy环境下.
## 1: 建图 & 定位 运行指南
**注意: 无论是定位还是建图，启动前务必 kill 掉所有残留节点**:
```bash
pkill -9 -f rviz2; pkill -9 -f 'ros2 launch'; pkill -9 -f 'ros2 bag play'; pkill -9 -f localizer_node; pkill -9 -f fastlivo_mapping; pkill -9 -f transformer_node; pkill -9 -f sc_pgo_node; pkill -9 -f align_monitor; sleep 3; ps -ef | grep -E 'rviz2|ros2 launch|ros2 bag|localizer|fastlivo|transformer|sc_pgo|align_monitor' | grep -v grep
```
确保无输出

### 🗺️ 1.1 启动建图（等待外部 topic）
作用：拉起 `FAST-LIVO2` 建图节点。节点会订阅 /livox/lidar 和 /livox/imu，外部一发数据就开始建图。
```bash
# 1. 进入 chroot 后，先 source 环境
source /opt/ros/foxy/setup.bash
source /root/fastlivo_ws/install/setup.bash
source /root/GO2_fastlivo2/install/setup.bash

# 2. 准备保存目录（以当前时间为准）
SAVE_DIR=/root/GO2_fastlivo2/maps_$(date +%Y%m%d_%H%M%S)
mkdir -p "$SAVE_DIR/Scans"
echo "本次地图将保存到: $SAVE_DIR"

# 3. 启动建图节点（前台运行，方便看日志，Ctrl+C 结束）
cd /root/GO2_fastlivo2
ros2 launch fast_livo mapping_mid360.launch.py \
  use_rviz:=False \
  avia_params_file:=/root/GO2_fastlivo2/install/fast_livo/share/fast_livo/config/mid360.yaml \
  save_dir:="$SAVE_DIR"
```
提示：建图时如果想看实时点云，可以另开终端手动运行 
```bash
source /opt/ros/foxy/setup.bash
source /root/fastlivo_ws/install/setup.bash
source /root/GO2_fastlivo2/install/setup.bash
rviz2 -d /root/GO2_fastlivo2/install/fast_livo/share/fast_livo/rviz_cfg/fast_livo2.rviz
```
建图结束后，按 `Ctrl+C` 关闭终端，地图会自动保存到 `$SAVE_DIR`。

### 🎯 1.2 启动定位（等待外部 topic + RViz 画箭头）
作用：拉起定位节点和 RViz，等待外部数据，并在 RViz 中通过画箭头完成初始化（autoMode:0）。
```bash
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
操作：启动后，等待外部终端开始发布 topic，在弹出来的 RViz 窗口顶部点击 `2D Pose Estimate` 按钮，在地图上机器人起点位置按住左键拖动出一个箭头（指向车头朝向），松开即可完成初始化。

## 2: 建图 & 定位 输入输出说明
### 2.1 🗺️ 建图程序 (FAST-LIVO2)
📥 输入 (Input)  
- `/livox/lidar`：雷达点云 (类型：livox_ros_driver2/msg/CustomMsg),10 Hz
- `/livox/imu`：IMU 数据 (类型：sensor_msgs/msg/Imu),200 Hz

📤 输出 (Output)  
- 位姿相关：/aft_mapped_to_init （雷达里程计位姿，建图过程中的相对轨迹）
- 点云相关：`/cloud_registered_local` （当前帧点云）、/Laser_map （累积的全局地图点云）
- 落盘保存：`GlobalMap.pcd` 和 Scans/ 关键帧文件夹

  **pcd 文件建议使用 pcl_viewer 看结果（本foxy环境下已装）**
  ```bash
  # pcl_viewer 看结果
  source /opt/ros/foxy/setup.bash
  pcl_viewer /root/GO2_fastlivo2/maps_20261104_104007/GlobalMap.pcd
  ```
### 2.2 🎯 定位程序 (Localizer)

📥 输入 (Input)  
- `/livox/lidar` 和 `/livox/imu`：实时雷达和 IMU 数据。
- `/initialpose`：初始位姿（类型：geometry_msgs/msg/PoseWithCovarianceStamped）。在 RViz 里点 “2D Pose Estimate” 画箭头，就是往这个话题发数据。
- `GlobalMap.pcd`：启动时加载的全局先验地图。

📤 输出 (Output) - 重点关注位姿
- 核心位姿：`/slam/lidar_pose` （类型：geometry_msgs/msg/PoseStamped 或带协方差的）。这是定位最终输出的全局位姿，**frame_id 是 map**。你在终端 echo 它 (`ros2 topic echo /slam/lidar_pose`)就能看到实时的 x/y/z 坐标和四元数姿态。  
  ```
  注：在 SLAM 里，`map` 指的是全局世界坐标系。它的定义完全取决于建图的那一次运行:
  - 原点通常是建图程序刚启动、收到第一帧雷达/IMU数据时，设备所在的位置。
  - X 轴：建图起点时，车头/设备的正前方；
  - Y 轴：建图起点时，车头/设备的左侧；
  - Z 轴：垂直向上（地平面）。
  ```
- 点云相关：`/slam/points` （基于全局地图对齐后的实时点云，RViz 里看的就是这个）。


# update 2026.11.11
2. 停止两份 Foxy 文件桥
在宿主机终端先核对这两个 PID 是否仍是 Foxy 文件桥：
```bash
sudo chroot /home/foxy_box /bin/bash -c \
'ps -p 1231411,1337654 -o pid=,args='
```
只有确认输出中的 `PID` 对应 `/mnt/go2-live/src/ros2/go2_file_bridge_wsl2.py` 后，才执行：
```sudo kill -INT 1231411 1337654```
若 PID 已变化，不要照用旧 PID；分别在两份桥接进程所在终端按 Ctrl+C。等待几秒后重新确认：
```bash
sudo chroot /home/foxy_box /bin/bash -c \
'ps -eo pid,args | grep "[g]o2_file_bridge_wsl2.py"'
```
这里应没有 Foxy 文件桥进程。宿主机 Humble 的 bridge 使用 domain 116，与 Foxy 的 117 隔离，不要把它误认为这两个 Foxy publisher。
3. 在一个 Foxy 终端启动唯一的文件桥
```bash
sudo chroot /home/foxy_box /bin/bash
```
进入 chroot 后执行：
```bash
source /opt/ros/foxy/setup.bash
source /root/fastlivo_ws/install/setup.bash
source /root/GO2_fastlivo2/install/setup.bash

export ROS_DOMAIN_ID=117
export ROS_LOCALHOST_ONLY=1
export GO2_DATA_DIR=/mnt/go2-live/data
export PYTHONPATH="/mnt/go2-live/src/ros2:${PYTHONPATH:-}"

python3 -c 'import rclpy; from livox_pointcloud import make_livox_pointcloud; print("Foxy imports OK")'
python3 /mnt/go2-live/src/ros2/go2_file_bridge_wsl2.py
```
保持该终端运行。不要再启动第二份 `go2_file_bridge_wsl2.py`。  
4. 用另一个 Foxy 终端确认 publisher 数量
再次进入 chroot，source 环境并设置相同 domain：
```bash
sudo chroot /home/foxy_box /bin/bash
source /opt/ros/foxy/setup.bash
source /root/fastlivo_ws/install/setup.bash
source /root/GO2_fastlivo2/install/setup.bash
export ROS_DOMAIN_ID=117
export ROS_LOCALHOST_ONLY=1
```
检查：
```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/imu
```
两者都应显示 `Publisher count: 1`，且 `publisher` 节点为 `go2_file_bridge`。若仍为 2，先不要启动建图；再检查是否有遗漏的 Foxy bridge：
```bash
ps -eo pid,args | grep "[g]o2_file_bridge_wsl2.py"
```
5. 重启 IMU 可视化节点和 RViz
另开 Foxy 终端，进入 chroot、source 环境、设置 ROS_DOMAIN_ID=117 和 ROS_LOCALHOST_ONLY=1，然后运行：
```bash
python3 /mnt/go2-live/src/ros2/livox_imu_rviz.py
```
再开一个 Foxy 终端，用相同环境启动 RViz：
```bash
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
rviz2 -d /mnt/go2-live/src/rviz/foxy_livox.rviz
```
6. 确认输入正常后启动建图
另开 Foxy 终端，进入 chroot、source 环境并设置相同 domain，然后检查：
```bash
ros2 topic info /livox/lidar
ros2 topic info /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```
确认两个话题都有 `publisher` 且持续收到消息后，按 Ctrl+C 停止各自的 topic hz，再启动建图：
```bash
cd /root/GO2_fastlivo2
SAVE_DIR="/root/GO2_fastlivo2/maps_check_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SAVE_DIR/Scans"

ros2 launch fast_livo mapping_mid360.launch.py \
  use_rviz:=False \
  avia_params_file:=/root/GO2_fastlivo2/install/fast_livo/share/fast_livo/config/mid360.yaml \
  save_dir:="$SAVE_DIR"
```
建图启动后，再检查 /cloud_registered_local 等输出。
# InteriorGS 第二阶段验收

第一阶段标签 `our-go2-d435-mid360-v1-validated` 指向 `dc524ee`，在本阶段开始前已经完成。第二阶段分支 `codex/our-go2-interiorgs`。

完整 GUI X11 六方向 WASDZC + 松键停止、实际物理运动、ROS控制回传及超时停止、高斯RGB、mesh深度/LiDAR、物理IMU、独立三角网格校验：53/53通过（validation-final.log）。随后依据同次实际ROS接收时间戳补充连续200Hz运输检查，合计54/54，原始2404个时间戳见ros_topics.json。可运行 python -m scripts.validate_imu_transport reports/interiorgs 重算。

32个随机种子的occupancy/净空及独立三角网格地板求交全部通过，见spawn_sweep.json。seed7、seed123另有实际100步物理及传感器冒烟记录（相邻reports目录），与主seed42共同覆盖三个实际出生点。附加出生检查仅为短时站立和传感器检查，六方向运动在seed42执行。

RGB读取原始USDZ中的1,445,429个NuRec高斯和SH3，由gsplat渲染。原生RTX相机/自定义RTX LiDAR在该资产上的投影/返回不满足独立几何核验，因此本阶段深度与LiDAR改用真实Warp CUDA三角形射线；物理接触仍由原USD的PhysX碰撞承载。未用灰度图、合成点云或高斯深度冒充mesh结果。

早期长ROS前进测试撞近障碍并翻倒，52/53（validation3.log）；室内完整复测将ROS发令窗口从100步改成30步，仍验收实际ROS移动及超时停止，且保持同一姿态/高度门槛，并额外加入机身up向量门槛。最终轨迹最小高度和倾角见results.json。此实现是键盘遥控与传感器仿真，不是避障控制器，不能保证持续向墙行走仍稳定。

外参沿用既有安装位置，尚未实机标定。RGB不合成机器人mesh；当前深度/LiDAR亦不包含机器人自身遮挡。扫描为瞬时程序化非重复覆盖近似；无厂商专有扫描轨迹、反射率、噪声或运动畸变。所有采样率按模拟时间，当前约0.06 RTF，非墙钟实时。Chromium实际渲染HTML并检查图像显示。

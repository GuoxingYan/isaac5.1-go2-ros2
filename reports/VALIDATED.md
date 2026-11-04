# Ubuntu 22.04 Go2 验收结果

2026-11-04，使用现有 isaacsim_510 环境完成迁移。

- 最终完整运行：reports/verified_run.log，退出码 0。
- 运行时检查：reports/latest/results.json，29/29 通过。
- 场景/多机器人：reports/scene_results.json，10/10 通过。
- 七个场景均实际加载并执行 30 步物理仿真；完整仓库 3470 Mesh。
- flat/rough 策略分别使用三机器人执行 100 步；编号/F1 切换、组合键和 SPACE 通过。
- 三机器人各自 RGBD、RTX LiDAR 均取得有效数据。
- 六方向真实 X11 窗口按键、命令方向、实际位移/转角与松键停止通过。
- 本轮 ROS2 端到端实际移动 1.157 米，并通过 0.5 秒命令超时停止。
- HTML 自包含真实 RGB、米制深度、语义、LiDAR、RGB-D 点云、轨迹和单/多机器人视图。
- Python compileall 与所有 Linux 启动脚本 bash -n 通过。

范围：场景为短时冒烟验收；未进行长时间耐久测试。最终完整测试平均 RTF 约 0.19，在 GUI、真实传感器与数据记录同时开启时并非实时。Kit 仍输出插值/旧 API 警告，已保留原始日志。默认 ROS 话题对应机器人 0，所有机器人状态位于文件元数据中。

最终 HTML 已通过 Chromium 无头浏览器截图复查，预览为 reports/report_preview.png。验收期间创建的 ROS bridge、probe、预览 HTTP 服务已停止，未停止工作站原有仿真进程。

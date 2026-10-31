# sentry_nav_071

从机器人 MiniPC 的 `nav_ws_IOT` 当前源码整理出的 RoboMaster 雷达建图与导航工作区。保留 Livox MID-360、Point-LIO、SLAM Toolbox、frontier 自主探索、Nav2、串口底盘速度桥及修改后的重定位算法；不包含云端、MQTT、相机、图像识别、视频流或 UWB 跟随功能。原始工作区未被覆盖。

## 环境和构建

目标环境为 Ubuntu 22.04 和 ROS 2 Humble。机器人描述依赖 `xmacro` 1.2.1；重定位的 Python 初始位姿辅助程序还需要 NumPy、SciPy、OpenCV 和 PyYAML，串口桥需要 pyserial。除常规 ROS 依赖外，需要 Nav2、SLAM Toolbox、PCL、OpenMP、Eigen、glog 等；可使用 `rosdep` 检查本机缺项。

```bash
cd ~/sentry_nav_071
source /opt/ros/humble/setup.bash
python3 -m pip install --user xmacro==1.2.1
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`small_gicp/` 固定为 MiniPC 使用的版本 `78f2e7a221720625eb95271ad9da21a04fb77f86`。Livox 驱动目录中保留 MiniPC 原有的 SDK 头文件及 amd64、arm64 动态库。当前机器人模型的雷达安装为相对车体中心 x=-0.07 m、y=0，离地 z=0.46 m，绕 X 轴倒装 3.14 rad；车体模型的 `chassis` 原点离地 0.077 m，因此雷达相对 `chassis` 的 z=0.383 m。Point-LIO 的雷达/内置 IMU 相对标定未修改。雷达 IP、网口和外参仍需上车确认；安装位置改变后应重新建图，不能假定旧地图仍匹配。仓库不包含运行时生成的 PGM、YAML 或 PCD 地图。

## 建图

启动前检查急停、雷达、串口设备和机器人周围安全。脚本默认开启自主探索和底盘串口，可能使机器人移动；`/dev/serial_port` 是原机器人串口路径。

```bash
cd ~/sentry_nav_071
./start_mapping.sh
```

按一次 Ctrl+C 并等待进程退出：地图记录节点写出缓存的 PGM/YAML，Point-LIO 在退出时写出 PCD。脚本检查二维地图、PCD 点数及数据长度，计算校验值，通过后才更新 `~/sentry_maps/latest.json`。每次建图使用新的独立目录；保存失败时保留上一次的有效记录。不要连续按 Ctrl+C 或直接断电、`kill -9`，这些方式不能保证保存。地图完整性检查不保证地图质量或定位精度。

## 导航与重定位

```bash
cd ~/sentry_nav_071
./start_navigation.sh
```

导航自动校验并加载最近一次成功保存的地图，并启动 PGM 初始位姿辅助程序。重定位保留 PGM 多角度候选匹配及 small_gicp 点云匹配、候选评分和多帧一致性校验；对称场景、遮挡、环境变化仍可能造成歧义。发目标点前必须在 RViz 检查点云和地图的平移、朝向是否吻合，不能仅凭节点启动判断定位正确。若需要手动指定初始位姿，可使用 RViz 的 `2D Pose Estimate`。地图文件被手动修改后，校验将拒绝自动加载，应重新建图或明确重新验证后更新记录。

两个脚本共享锁，防止通过它们同时启动建图和导航，但不会停止其他方式启动的旧进程。可用环境变量 `SENTRY_MAP_ROOT` 指定地图目录、`SENTRY_SERIAL_PORT` 指定串口、`SENTRY_USE_RVIZ=False` 关闭 RViz；调试时可设 `SENTRY_ENABLE_MCU_SERIAL=False` 断开软件串口桥、`SENTRY_AUTO_EXPLORE=False` 禁止自动探索。这些选项不能替代物理急停或断开驱动。

## 上车前检查

实际速度、串口协议、雷达外参、机器人足迹、膨胀层、制动距离和地图均沿用原机器人配置，不能直接假定适配其他 RoboMaster 底盘。首次验证应让驱动轮离地，检查 `/cmd_vel` 的方向和量级、串口超时停车，再低速分段测试避障、窄通道和重定位；急停必须独立可用。软件检查或构建通过不等于比赛场地的实车安全验证。

第三方代码及资源的版权、许可证保留在各自目录，包括 `src/pb2025_sentry_nav/LICENSE`、`src/m-explore/LICENSE` 和 `small_gicp/LICENSE`。

本地静态及重定位几何回归测试（需要 NumPy、SciPy）：

```bash
python3 -m unittest discover -s tests -v
```

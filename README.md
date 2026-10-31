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

新增仿真包后，已有工作区也需要重新运行 `rosdep install` 和 `colcon build`。仿真使用随仓库保存的 Nav2 `nav2_loopback_sim` Humble 适配版本，额外依赖 `tf_transformations`、`transforms3d` 等，由 rosdep 安装。

`small_gicp/` 固定为 MiniPC 使用的版本 `78f2e7a221720625eb95271ad9da21a04fb77f86`。Livox 驱动目录中保留 MiniPC 原有的 SDK 头文件及 amd64、arm64 动态库。当前机器人模型的雷达安装为相对车体中心 x=-0.07 m、y=0，离地 z=0.52 m，绕 X 轴倒装 3.14 rad；车体模型的 `chassis` 原点离地 0.077 m，因此雷达相对 `chassis` 的 z=0.443 m。Point-LIO 的雷达/内置 IMU 相对标定未修改。雷达 IP、网口和外参仍需上车确认；安装位置改变后应重新建图，不能假定旧地图仍匹配。仓库不包含运行时生成的 PGM、YAML 或 PCD 地图。

## 建图

启动前检查急停、雷达、串口设备和机器人周围安全。脚本默认开启自主探索和底盘串口，可能使机器人移动；`/dev/serial_port` 是原机器人串口路径。

```bash
cd ~/sentry_nav_071
./start_mapping.sh --check
./start_mapping.sh
```

`--check` 只检查已安装的建图程序、Python 依赖、实际机器人模型展开和地图目录写入，不启动机器人节点；它不代替雷达、IMU、TF 的现场验证。检查失败时先修复依赖或重新编译，再开启建图。

按一次 Ctrl+C 并等待进程退出：地图记录节点写出缓存的 PGM/YAML，Point-LIO 在退出时写出 PCD。停止请求由独立的会话管理程序转发给 ROS launch；向启动进程 PID 发送 SIGTERM 也会转为正常退出，等待保存后再释放会话锁。脚本检查二维地图、PCD 点数及数据长度，计算校验值，通过后才更新 `~/sentry_maps/latest.json`。每次建图使用新的独立目录；保存失败时保留上一次的有效记录。不要直接断电或使用 `kill -9`，这些方式不能保证保存。地图完整性检查不保证地图质量或定位精度。

自主探索启动程序会缓存已发布的就绪地图，在启动延时结束后检查稳定状态，不依赖重新收到三张地图。持续地图有效性检查与探索启动条件分开，未知区域减少到零时仍向 Nav2 更新合法地图，避免探索后期沿用旧地图。

## 使用现场地图做 RViz 导航仿真

现场建图正常结束、看到 PGM/YAML 和 PCD 保存成功后运行：

```bash
./start_navigation_sim.sh
```

脚本自动校验并加载 `~/sentry_maps/latest.json` 指向的最近一次完整地图，启动 RViz、原机器人模型、Nav2 规划与控制、虚拟底盘和地图扫描。先在 RViz 用 **2D Pose Estimate** 在可通行区域放置虚拟机器人并指定朝向，再用 **Nav2 Goal** 点击目标，观察路线和虚拟机器人移动。默认仿真允许自由发送 RViz 目标。

也可以一次启动前面相同的双目标任务：

```bash
./start_navigation_sim.sh 1.0 2.0 4.0 2.5
```

或使用已配置的 `config/two_point_route.yaml`：

```bash
./start_navigation_sim.sh --two-point
```

仍先在 RViz 设置虚拟初始位姿；地图、TF 和 Nav2 就绪后自动执行第一点 0.5 m、第二点 0.2 m 的流程。坐标示例需换成现场地图中的实际点。仿真与实车使用相同的规划器、控制器、速度平滑、机器人半径与膨胀设置；仿真覆盖文件将地形点云层替换为地图与虚拟扫描层，不改动实车参数文件。

仿真默认使用独立 **ROS_DOMAIN_ID=71**，速度输出为 `sim/cmd_vel`，并且只启动虚拟传感器和底盘。脚本拒绝与当前终端的实车 DDS 域相同的配置；可通过 `SENTRY_SIM_DOMAIN_ID` 选择其他域。通过命令行观察仿真话题时，例如：

```bash
ROS_DOMAIN_ID=71 ros2 topic echo /odometry
```

建图、实车导航和仿真脚本共享会话锁；先用 Ctrl+C 正常结束当前模式，再启动另一模式。仿真退出同样使用启动终端的 Ctrl+C。仿真采用墙钟时间，不需要 `/clock`。

这是地图上的二维运动仿真，可验证地图加载、路径规划、全向移动和双点顺序；不会模拟车轮打滑、惯性、实体碰撞、移动行人或雷达噪声，也不能验证三维 PCD 重定位精度。模型沿用真实雷达安装姿态；虚拟二维扫描使用单独的水平 `sim_scan` 坐标系，避免倒装雷达使模拟扫描左右翻转。现场建图和实车导航效果仍需在 MiniPC 上运行验证。

## 一键导航与双点任务

一个命令完成加载最新地图、启动导航和 RViz、等待重定位、依次经过两个目标点。四个数字依次为 `x1 y1 x2 y2`，使用 `map` 坐标系，单位为米；下面坐标仅为格式示例，应换成实际目标：

```bash
cd ~/sentry_nav_071
./start_navigation.sh 1.0 2.0 4.0 2.5
```

上述示例先前往 `(1.0, 2.0)`，进入第一点的 **0.5 m** 范围后取消第一段目标，确认结束后切换到 `(4.0, 2.5)`；进入第二点的 **0.2 m** 范围后取消当前目标并结束任务，不循环、不返回、不要求最终朝向。导航和 RViz 继续运行，但任务节点不再发送目标。无需再开第二个终端执行双点脚本。

也可以先在工作区根目录的 `config/two_point_route.yaml` 中保存坐标：

```yaml
first: [1.0, 2.0]
second: [4.0, 2.5]
```

之后只需运行 `./start_navigation.sh`。仓库中的四个初始值为 `null`，必须设置成实际坐标；未填写、数量不对、NaN 或无穷数都会在启动机器人节点前报错。命令行四个坐标优先于配置文件，也可用 `SENTRY_ROUTE_CONFIG` 指定其他配置文件。坐标应对应当前加载的地图，重新建图后需重新检查两个点。

导航自动校验并加载最近一次成功保存的地图，并启动 PGM 初始位姿辅助程序。双点任务等待 small_gicp 接受点云配准、PGM/手动初始位姿完成地图坐标对齐且连续 1 秒未更新、Nav2 生命周期进入 active、位置 TF 有效后才下发第一点；初始的单位 TF 不算定位成功。等待没有固定超时，会定期打印状态。PGM 辅助定位失败时，可以在 RViz 使用 `2D Pose Estimate` 给出正确初始位姿。

重定位保留 PGM 多角度候选匹配及 small_gicp 点云匹配、候选评分和多帧一致性校验；修复首次仅精配准成功时绕过初始一致性校验、导致地图种子一直不被应用的状态问题，配准算法和质量阈值不变。`localization_ready` 同时要求最近 2 秒内有扫描、最近 10 秒内有被接受的配准，不会仅因定时发布旧 TF 而一直有效。它不保证对称场景、遮挡、环境变化下绝无定位歧义；运行时仍应观察 RViz 中点云与地图是否吻合。地图文件被手动修改后，校验将拒绝自动加载，应重新建图或明确重新验证后更新记录。

三个启动脚本共享锁，防止通过它们同时启动建图、实车导航和仿真，但不会停止其他方式启动的旧进程。可用环境变量 `SENTRY_MAP_ROOT` 指定地图目录、`SENTRY_SERIAL_PORT` 指定串口、`SENTRY_USE_RVIZ=False` 关闭 RViz；调试时可设 `SENTRY_ENABLE_MCU_SERIAL=False` 断开软件串口桥、`SENTRY_AUTO_EXPLORE=False` 禁止自动探索。这些选项不能替代物理急停或断开驱动。

## 全向移动与受阻重试

导航和建图默认使用全向移动，保留底盘坐标下的 `vx`、`vy`，不需要先转向运动方向。Nav2 的速度平滑仍生效。常规启动入口不再按导航模式强制启用 `forward_only_enabled`。接近终点的最低线速度为 0.30 m/s，沿用此前测得的底盘最低有效速度，避免全向模式不再经过前向死区补偿后卡在到达范围外；实际速度与制动仍需上车核验。

到达判定依据是 `map -> gimbal_yaw_fake` 的实时位置，因此容差是定位系统测得的距离，实际停车误差还受定位误差、底盘制动和控制周期影响。

双点任务使用独立的 `two_point_goal_checker`，普通单点导航仍使用原来的 0.35 m 到达容差。控制器的终点停车范围为 0.15 m，位于双点任务 0.2 m 容差内，避免在任务完成范围外提前停住。若 Nav2 将目标投影到邻近安全点，脚本仍按原始坐标检查距离，不能把范围外的投影点当成完成。

第一点完成前不会发送第二点。取消每点 300 秒期限，导航行为树在规划或跟踪失败后等待 1 秒并重新规划同一目标；即使 Nav2 意外返回 aborted，任务节点也保留原目标并重试。临时障碍消失后继续，不因耗时、无进展或累计失败次数拉黑目标。即使 Nav2 提前报告成功，原始坐标仍在到达范围外时也会继续尝试该点，不跳过。永久无法到达的目标会一直重试，需要用户明确取消或修改坐标。

探索节点也移除了“长期无进展拉黑”和“多次 aborted 后拉黑/排除附近坐标”的代码。仍保留无可用安全导航栅格时切换其他 frontier 的几何筛选，以及控制器进度检测、障碍和串口超时停车。进度检测失败只触发本轮重规划，不让任务遗忘目标。

按 Ctrl+C 退出当前启动进程；明确取消不会自动重启双点任务。定位 TF 或就绪信号持续不可用超过 2 秒时取消当前动作、保留目标并等待重新定位，就绪后继续同一点；不会用过期位置判断到达。目标拒绝、动作响应通信异常、无法确认取消时仍会结束任务并报告错误，不发送第二点。运行期间不要从 RViz 或其他客户端发送竞争目标。任务节点只下发 Nav2 目标，不直接发布底盘速度。

## 上车前检查

实际速度、串口协议、雷达外参、机器人足迹、膨胀层、制动距离和地图均沿用原机器人配置，不能直接假定适配其他 RoboMaster 底盘。首次验证应让驱动轮离地，检查 `/cmd_vel` 的方向和量级、串口超时停车，再低速分段测试避障、窄通道和重定位；急停必须独立可用。软件检查或构建通过不等于比赛场地的实车安全验证。

第三方代码及资源的版权、许可证保留在各自目录，包括 `src/pb2025_sentry_nav/LICENSE`、`src/m-explore/LICENSE`、`src/nav2_loopback_sim/LICENSE` 和 `small_gicp/LICENSE`。仿真器上游来源与本地适配记录见 `src/nav2_loopback_sim/README.md`。

本地静态及重定位几何回归测试（需要 NumPy、SciPy）：

```bash
python3 -m unittest discover -s tests -v
```

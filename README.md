# sentry_nav_071

面向 RoboMaster 的 Livox MID-360 建图、重定位与全向底盘导航工作区，从 MiniPC 的 `nav_ws_IOT` 源码整理而来。

当前使用流程是：**人工控制机器人建图，退出时保存 PGM/YAML/PCD，再使用新地图进行实车导航或 RViz 导航仿真。自主探索默认关闭。** 实车导航脚本会自动执行两个目标点：第一点进入 0.5 m 范围后切换，第二点进入 0.2 m 范围后结束任务。

## 目录

- [1. 功能与命令速查](#1-功能与命令速查)
- [2. 运行环境](#2-运行环境)
- [3. 克隆项目](#3-克隆项目)
- [4. 安装依赖](#4-安装依赖)
- [5. 编译和更新](#5-编译和更新)
- [6. 雷达与底盘配置](#6-雷达与底盘配置)
- [7. 人工控制建图](#7-人工控制建图)
- [8. 保存地图与切换地图](#8-保存地图与切换地图)
- [9. 实车导航与双目标任务](#9-实车导航与双目标任务)
- [10. 使用现场地图进行导航仿真](#10-使用现场地图进行导航仿真)
- [11. 单独打开或关闭 RViz](#11-单独打开或关闭-rviz)
- [12. 停止进程与重新启动](#12-停止进程与重新启动)
- [13. 参数与环境变量](#13-参数与环境变量)
- [14. 运行状态检查与故障排查](#14-运行状态检查与故障排查)
- [15. 代码结构与测试](#15-代码结构与测试)

## 1. 功能与命令速查

以下命令在项目根目录执行，首次使用前先完成第 2 至第 6 节。

| 命令 | 功能 | 是否控制实车 |
| --- | --- | --- |
| `./start_mapping.sh --check` | 检查建图程序、Python 依赖、机器人模型和地图目录 | 否，不启动机器人节点 |
| `./start_mapping.sh` | 启动建图和 RViz，由人工控制机器人移动 | 串口桥默认开启；不自动生成探索目标 |
| `SENTRY_ENABLE_MCU_SERIAL=False ./start_mapping.sh` | 建图时不启动本项目的底盘串口桥，便于其他控制程序或下位机遥控 | 本项目不发送串口速度 |
| `./start_navigation.sh x1 y1 x2 y2` | 加载最新完整地图，重定位就绪后自动执行双目标任务 | 是 |
| `./start_navigation.sh` | 从 `config/two_point_route.yaml` 读取双目标坐标并执行 | 是，必须先填写配置 |
| `./start_navigation_sim.sh` | 加载最新完整地图，使用 RViz 点击目标进行仿真 | 否，虚拟底盘 |
| `./start_navigation_sim.sh x1 y1 x2 y2` | 在仿真中自动执行双目标任务 | 否 |
| `./start_navigation_sim.sh --two-point` | 在仿真中执行配置文件中的双目标任务 | 否 |
| 三个启动脚本的 `--help` | 查看各自支持的命令格式 | 否 |
| 启动终端按一次 `Ctrl+C` | 正常退出；建图模式还会保存地图 | 退出当前会话 |

查看入口自带帮助：

```bash
./start_mapping.sh --help
./start_navigation.sh --help
./start_navigation_sim.sh --help
```

三个启动脚本共享会话锁。同一地图根目录下，一次只运行建图、实车导航、仿真中的一个模式。建图结束不会自动切换导航，需要运行相应的导航脚本。

## 2. 运行环境

本文面向以下环境：

| 项目 | 要求或当前配置 |
| --- | --- |
| 系统 | Ubuntu 22.04，本文命令使用 Bash |
| ROS | 已安装 ROS 2 Humble，默认路径 `/opt/ros/humble` |
| MiniPC | 当前以 x86_64 平台的 ASUS NUC 为部署对象 |
| 雷达 | Livox MID-360，使用内置 IMU |
| 底盘 | 全向底盘，允许 `vx`、`vy` 和 `wz`，无需先朝向运动方向 |
| 串口 | 默认 `/dev/serial_port`，115200、8N1 |
| 可视化 | RViz，需要可访问的图形桌面会话 |
| 默认地图目录 | `~/sentry_maps`，独立于 Git 仓库 |

先确认 ROS 环境存在：

```bash
test -r /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

最后一条应输出 `humble`。若缺少 `setup.bash`，先安装适用于 Ubuntu 22.04 的 ROS 2 Humble；下面的 ROS 软件包安装命令以 ROS 软件源已配置为前提。建议使用系统 Python 3，避免在 Conda 环境中编译或启动 ROS。

## 3. 克隆项目

系统尚未安装 Git 时先执行：

```bash
sudo apt update
sudo apt install -y git
```

然后克隆：

```bash
cd ~
git clone https://github.com/zzzAI999/sentry_nav_071.git
cd ~/sentry_nav_071
git branch --show-current
```

默认分支为 `main`。下文统一使用 `~/sentry_nav_071`，若克隆到了其他位置，替换相应路径。

仓库已经包含 `small_gicp/` 源码和 Livox SDK 头文件、动态库，不需要再次克隆这两个依赖，也不需要执行子模块下载。建图产生的 PGM、YAML 和 PCD 不随代码仓库分发。

## 4. 安装依赖

### 4.1 安装基础工具和主要依赖

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake pkg-config util-linux psmisc \
  python3-pip python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-opencv python3-yaml python3-serial \
  libeigen3-dev libpcl-dev libgoogle-glog-dev libunwind-dev libapr1-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-rviz2 \
  ros-humble-tf-transformations ros-humble-joint-state-publisher
```

机器人描述需要 `xmacro` 1.2.1：

```bash
python3 -m pip install --user xmacro==1.2.1
```

### 4.2 使用 rosdep 补齐各 ROS 包的依赖

首次使用 rosdep 时初始化；已经初始化的机器直接执行后面的 `rosdep update`：

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
cd ~/sentry_nav_071
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

检查是否仍有缺项：

```bash
rosdep check --from-paths src --ignore-src
```

`nav2_loopback_sim` 是仓库内的仿真包，`tf_transformations`、`transforms3d` 等依赖也由上述步骤安装。若 rosdep 报告缺少依赖或无法解析的键，先解决该错误，再进行编译。

## 5. 编译和更新

### 5.1 首次编译

```bash
cd ~/sentry_nav_071
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
./start_mapping.sh --check
```

`--base-paths src` 将 ROS 包的发现范围限定在 `src/`。工作区根目录的 `small_gicp/` 由重定位包的 CMake 自动引入，不需要单独运行一次 colcon 或安装其 Python 绑定。

如果编译时内存不足，可用串行执行和较低的单包编译并发重试：

```bash
MAKEFLAGS="-j2 -l2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src \
  --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 5.2 后续更新代码

先正常退出正在运行的会话，再更新和重新编译：

```bash
cd ~/sentry_nav_071
git status --short
git pull --ff-only
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
./start_mapping.sh --check
```

如果 `git pull` 因本地修改而失败，先检查并保存自己的修改，不要直接覆盖。地图放在 `~/sentry_maps`，正常拉取和编译不会改写地图。

### 5.3 新终端的环境

三个 `.sh` 入口会自行加载 ROS 和工作区环境。手动运行 `ros2` 排查命令或直接调用 launch 时，新终端先执行：

```bash
cd ~/sentry_nav_071
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 6. 雷达与底盘配置

### 6.1 MID-360 网络

当前配置文件为 [mid360_user_config.json](src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json)：

| 项目 | 当前值 |
| --- | --- |
| MiniPC 接雷达的有线网卡 IPv4 | `192.168.1.50` |
| 雷达 IPv4 | `192.168.1.113` |
| 雷达侧命令、推送、点云、IMU 端口 | `56100`、`56200`、`56300`、`56400` |
| 主机侧对应接收端口 | `56101`、`56201`、`56301`、`56401` |

SSH 或 NoMachine 使用的 Wi-Fi 地址可以是其他网段；雷达有线网口仍需与此 JSON 配置一致。查看网卡和测试雷达：

```bash
ip -br address
ping -c 4 192.168.1.113
```

若雷达网口尚未配置地址，可以先临时添加。以下 `enp1s0` 是示例，必须替换为 `ip -br address` 中实际接雷达的网卡名；已有该地址时不用重复添加：

```bash
LIDAR_IFACE=enp1s0
sudo ip address add 192.168.1.50/24 dev "$LIDAR_IFACE"
sudo ip link set "$LIDAR_IFACE" up
```

以上地址配置只用于当前开机周期，长期使用请在系统网络设置中配置静态 IPv4。更换雷达地址或主机有线地址后，修改 JSON 中的 `lidar_configs` 和相应 `host_net_info`，然后重新编译、重新启动。`ping` 通只说明网络可达，还要用第 14 节的命令确认点云与 IMU 正在发布。

### 6.2 雷达安装外参

当前机器人模型 [pb2025_sentry_robot.sdf.xmacro](src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro) 使用：

| 项目 | 当前值 |
| --- | --- |
| 相对车体中心 X | `-0.07 m` |
| 相对车体中心 Y | `0 m` |
| 雷达坐标原点离地高度 | `0.52 m` |
| 绕 X 轴倒装 | `3.14 rad`，约 180° |
| `chassis` 原点离地高度 | `0.077 m` |
| 雷达相对 `chassis` 的 Z | `0.443 m` |
| 机器人半径 | `0.32 m` |

车体安装外参与 Point-LIO 中雷达/内置 IMU 的相对标定是两件事。不要把离地高度直接填写到 `mapping.extrinsic_T`。安装位置变化后重新建图，并核对 RViz 中的模型、点云和地面方向。

### 6.3 底盘串口

查看当前设备和串口权限：

```bash
ls -l /dev/serial_port
ls -l /dev/serial/by-id/
groups
```

`/dev/serial_port` 是原机器上的设备路径或别名，新机器不一定存在。可通过环境变量指定真实设备，不必建立同名别名：

```bash
SENTRY_SERIAL_PORT=/dev/ttyUSB0 ./start_mapping.sh
SENTRY_SERIAL_PORT=/dev/ttyUSB0 ./start_navigation.sh 1.0 2.0 4.0 2.5
```

两个命令是不同会话的使用示例，不要同时启动；设备名和目标坐标均需替换。长期使用优先选择 `/dev/serial/by-id/` 下的稳定路径。

如果当前用户没有串口访问权限：

```bash
sudo usermod -aG dialout "$USER"
```

退出当前登录并重新登录后生效。串口桥默认以 50 Hz 发送速度，接收 `/cmd_vel`；超过 0.5 秒没有新速度命令时发送停车速度。实测最低有效线速度和角速度补偿目前为 `0.3 m/s`、`0.1 rad/s`，因此小的非零上层指令可能在串口前被抬到最低有效值；可检查 `/mcu/cmd_vel_effective`。

## 7. 人工控制建图

### 7.1 检查后启动

```bash
cd ~/sentry_nav_071
./start_mapping.sh --check
./start_mapping.sh
```

`--check` 只检查已安装的建图程序、Python 依赖、实际机器人模型展开和地图目录写入能力，不启动节点或驱动机器人。正常建图会打开 RViz、启动雷达、Point-LIO、二维 SLAM 和地图缓存，自主探索默认关闭，不自动发送探索目标。

启动后先保持机器人静止，等 IMU 初始化和点云显示正常，再由人工控制机器人缓慢移动采集场地。在 RViz 中观察 `/map` 是否随移动扩展、点云是否与墙面吻合。建图时不需要填写双目标坐标。

### 7.2 由其他程序或下位机直接遥控建图

本项目串口桥默认仍会运行，包括发送超时停车速度。若遥控由其他程序或下位机直接接管，应关闭本项目串口桥后再启动建图，避免同时使用同一控制链路：

```bash
SENTRY_ENABLE_MCU_SERIAL=False ./start_mapping.sh
```

此时雷达、IMU、建图、RViz 和退出保存仍运行，本项目不打开底盘串口。仓库未提供绑定某款手柄或遥控器的程序，人工移动命令由实际底盘控制系统提供。

### 7.3 不打开 RViz 的建图

```bash
SENTRY_USE_RVIZ=False ./start_mapping.sh
```

需要同时关闭串口桥时，可将环境变量写在同一条命令前：

```bash
SENTRY_USE_RVIZ=False SENTRY_ENABLE_MCU_SERIAL=False ./start_mapping.sh
```

### 7.4 结束建图

先停止人工移动，然后在启动建图的终端按一次 `Ctrl+C`，等待进程和保存完成。成功时终端会显示类似：

```text
Map saved: /home/用户名/sentry_maps/map_时间_随机后缀/map.yaml
Point cloud saved: /home/用户名/sentry_maps/map_时间_随机后缀/map.pcd
The next navigation or simulation start will use this validated map.
```

不要在保存过程中直接断电或使用 `kill -9`。退出后的下一步由你选择：第 9 节的实车导航，或第 10 节的导航仿真。

## 8. 保存地图与切换地图

### 8.1 保存位置和自动加载规则

每次建图使用新目录，默认结构如下：

```text
~/sentry_maps/
├── latest.json
├── .navigation.lock
├── map_时间_随机后缀/
│   ├── map.pgm
│   ├── map.yaml
│   └── map.pcd
└── 其他历史建图目录/
```

| 文件 | 用途 |
| --- | --- |
| `map.pgm` | 二维栅格图像 |
| `map.yaml` | 图像文件名、分辨率、地图原点等地图配置 |
| `map.pcd` | 三维点云地图，实车重定位使用 |
| `latest.json` | 最近一次成功保存并校验的地图记录，含路径、大小和 SHA-256 |

二维地图记录节点在退出时保存缓存的 PGM/YAML，Point-LIO 在退出时保存 PCD。会话管理程序等待 ROS launch 退出后再检查文件，只有完整性检查通过才更新 `latest.json`；保存失败时保留前一次有效记录。检查包括可读取的地图及有效自由栅格、二进制 PCD 的字段与数据长度、至少 1000 个点等；文件完整不代表地图几何质量合格。

实车导航和仿真脚本都读取这个记录，并再次校验文件。它们选择的是“最近一次完整保存并选中的地图”，不是简单按文件夹时间排序。仿真使用 PGM/YAML，但当前入口仍要求该次建图的 PCD 完整。

### 8.2 查看当前选中的地图

```bash
cd ~/sentry_nav_071
python3 -m json.tool "${SENTRY_MAP_ROOT:-$HOME/sentry_maps}/latest.json"
python3 tools/map_manifest.py select --manifest "${SENTRY_MAP_ROOT:-$HOME/sentry_maps}/latest.json"
```

第二条命令校验成功后依次输出 YAML、PCD 的绝对路径，失败则输出原因并返回非零状态。

### 8.3 使用自定义地图根目录

先让三个模式使用同一地图根目录：

```bash
export SENTRY_MAP_ROOT="$HOME/competition_maps"
./start_mapping.sh
```

保存并退出后，在同一环境中执行 `./start_navigation.sh ...` 或 `./start_navigation_sim.sh`。新终端也要设置同样的 `SENTRY_MAP_ROOT`，否则会使用默认的 `~/sentry_maps`。

### 8.4 手动选择一套历史地图

先停止当前会话。以下路径中的 `map_实际历史目录` 要替换为已有目录；YAML、PGM 和 PCD 必须属于同一次建图，并保持 YAML 对本目录 PGM 的引用：

```bash
cd ~/sentry_nav_071
python3 tools/map_manifest.py record \
  --manifest "${SENTRY_MAP_ROOT:-$HOME/sentry_maps}/latest.json" \
  --map "$HOME/sentry_maps/map_实际历史目录/map.yaml" \
  --pcd "$HOME/sentry_maps/map_实际历史目录/map.pcd"
```

成功后，下一次导航和仿真将使用这套地图。这个命令会重新校验文件并更新记录，不负责把不同来源的二维图和三维图对齐。不要仅修改 `latest.json` 中的路径或校验值；地图被改动、移动或复制后，应确认配套关系，再使用此命令重新登记。

## 9. 实车导航与双目标任务

### 9.1 一个命令启动导航并经过两点

先正常结束建图，确认地图已保存。四个数字按顺序为 `x1 y1 x2 y2`，使用 `map` 坐标系，单位为米：

```bash
cd ~/sentry_nav_071
./start_navigation.sh 1.0 2.0 4.0 2.5
```

上述坐标只是格式示例，必须换成现场地图中的实际可通行点。一个入口完成加载最新地图、打开 RViz、启动重定位和 Nav2、等待就绪，再执行任务，不需要额外运行双目标脚本。

| 阶段 | 行为 |
| --- | --- |
| 等待就绪 | 等待接受有效点云配准、地图坐标对齐稳定、Nav2 激活和实时位置 TF |
| 第一目标 | 前往 `(x1, y1)`，距离不超过 `0.5 m` 时结束第一段动作并切换 |
| 第二目标 | 前往 `(x2, y2)`，距离不超过 `0.2 m` 时结束动作并复核位置 |
| 任务结束 | 停在第二点附近，不循环、不返回；导航和 RViz 进程继续运行 |

到达范围依据 `map -> gimbal_yaw_fake` 测得的位置判定，不要求最终朝向。实际停车误差还受重定位误差、底盘制动和控制周期影响。普通单点目标默认容差为 `0.35 m`，双目标任务使用独立的 `0.20 m` 目标检查器。

### 9.2 将两个目标保存到配置文件

编辑 [config/two_point_route.yaml](config/two_point_route.yaml)：

```yaml
first: [1.0, 2.0]
second: [4.0, 2.5]
```

配置好后只需：

```bash
./start_navigation.sh
```

仓库初始坐标为 `null`，未填写会在启动节点前报错。命令行坐标优先于配置文件。换地图后重新核对两个点；不能使用 RViz 窗口像素坐标代替地图坐标。

如需从 RViz 读取坐标，可以添加 `Publish Point` 工具并点击地图，在另一终端执行 `ros2 topic echo /clicked_point --once`。仿真环境的该终端还需使用相同 ROS 域。

使用另一份配置文件：

```bash
SENTRY_ROUTE_CONFIG="$HOME/my_route.yaml" ./start_navigation.sh
```

只校验坐标、不启动 ROS 或机器人：

```bash
python3 src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_two_point_route.py \
  --print-points --config config/two_point_route.yaml
```

### 9.3 重定位、全向运动与临时受阻

重定位保留 PGM 多角度初始位姿匹配，以及 small_gicp 的点云配准、候选评分、多帧一致性和结果平滑。`localization_ready` 要求地图对齐后至少稳定 1 秒、最近 2 秒内有扫描、最近 10 秒内有接受的配准；初始单位 TF 不算重定位完成。

若自动初始位姿不正确，在 RViz 使用 **2D Pose Estimate** 在机器人真实位置点击并拖出实际朝向，观察点云是否与地图重合。对称走廊、重复结构、遮挡和场地变化仍可能造成定位歧义，不能把某次配准接受当成所有场景都能精确定位的保证。

全向底盘保留 `vx`、`vy`，默认不启用必须先朝前才能移动的限制，Nav2 速度平滑仍生效。双目标导航不因目标耗时、无进展或累计 aborted 次数拉黑目标；临时障碍引起失败后保留当前目标并重试。永久不可达的点会继续重试，需要人工取消或修改。

定位或新鲜 TF 持续不可用超过 2 秒时，任务取消当前动作、保留目标，等待恢复后继续。用户明确取消、目标被拒绝、通信异常或无法确认动作取消等情况仍会结束任务并报告错误，不会盲目发送第二点。双目标运行期间不要再从 RViz 或另一个客户端发送竞争目标。

### 9.4 仅启动导航，之后手动点击目标

标准 `start_navigation.sh` 固定启用双目标任务。如果只需重定位、Nav2 和 RViz，使用下面的进阶命令自动读取并验证当前地图，不启动双目标任务：

```bash
cd ~/sentry_nav_071
source /opt/ros/humble/setup.bash
source install/setup.bash
(
  set -e
  selected_map="$(python3 tools/map_manifest.py select \
    --manifest "${SENTRY_MAP_ROOT:-$HOME/sentry_maps}/latest.json")"
  mapfile -t saved_paths <<< "$selected_map"
  ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    mode:=navigation use_robot_state_pub:=True use_rviz:=True \
    map:="${saved_paths[0]}" prior_pcd_file:="${saved_paths[1]}" \
    enable_two_point_route:=False enable_auto_explore:=False \
    enable_mcu_status_nav:="${SENTRY_ENABLE_MCU_SERIAL:-True}" \
    mcu_serial_port:="${SENTRY_SERIAL_PORT:-/dev/serial_port}"
)
```

重定位正确后使用 RViz 的 **Nav2 Goal** 发送目标。直接运行 launch 不参与三个 `.sh` 的会话锁，应先结束其他模式，按 `Ctrl+C` 结束此会话。

## 10. 使用现场地图进行导航仿真

### 10.1 启动并在 RViz 点击目标

建图已完整保存并退出后：

```bash
cd ~/sentry_nav_071
./start_navigation_sim.sh
```

仿真自动加载同一个 `latest.json` 指向的地图，先启动地图服务、机器人模型、虚拟底盘和 RViz；放置虚拟机器人后再启动 Nav2。

1. 用 **2D Pose Estimate** 在地图可通行区域放置虚拟机器人，拖动箭头指定朝向。
2. 等待终端出现 `Simulation Nav2 active`，RViz 的 Navigation 状态变为 Active。
3. 用 **Nav2 Goal** 点击目标，观察规划路径和虚拟机器人运动。
4. 结束时在启动终端按 `Ctrl+C`。

刚启动时没有虚拟初始位姿，Nav2 会保持等待，不会开始规划或执行双目标任务。每次重新启动仿真都需要重新设置 **2D Pose Estimate**；箭头应放在本次地图的已知可通行区域。仅打开地图或点击 **Nav2 Goal** 不能代替设置初始位姿。

### 10.2 仿真执行双目标任务

直接填写两个点：

```bash
./start_navigation_sim.sh 1.0 2.0 4.0 2.5
```

或者读取 `config/two_point_route.yaml`：

```bash
./start_navigation_sim.sh --two-point
```

两种方式都需要先在 RViz 设置虚拟初始位姿，之后自动执行第一点 `0.5 m`、第二点 `0.2 m` 的任务。使用自定义配置文件时：

```bash
SENTRY_ROUTE_CONFIG="$HOME/my_route.yaml" ./start_navigation_sim.sh --two-point
```

### 10.3 ROS 域与仿真范围

仿真默认使用独立的 `ROS_DOMAIN_ID=71`，速度输出为 `/sim/cmd_vel`，不启动真实雷达和底盘串口。脚本拒绝使用与当前终端实车 ROS 域相同的域；也可以指定其他域：

```bash
SENTRY_SIM_DOMAIN_ID=72 ./start_navigation_sim.sh
```

用其他终端查看仿真时，在该条命令前设置相同的域，例如默认仿真：

```bash
ROS_DOMAIN_ID=71 ros2 node list
ROS_DOMAIN_ID=71 ros2 topic echo /odometry --once
ROS_DOMAIN_ID=71 ros2 topic echo /sim/cmd_vel
```

不必将整个终端永久 `export ROS_DOMAIN_ID=71`，否则再启动默认仿真脚本时，它会因检测到相同域而拒绝运行。若修改了 `SENTRY_SIM_DOMAIN_ID`，上面的检查命令也改为对应数字。

仿真复用实车规划器、控制器、速度平滑、半径和膨胀配置，通过覆盖文件把地形点云层替换为地图与虚拟扫描层。模型沿用实际雷达安装姿态，虚拟扫描使用水平的 `sim_scan` 坐标系，避免倒装雷达使模拟扫描翻转。仿真使用墙钟时间，不依赖 `/clock`。

这是二维运动学仿真，适合验证地图、路径、全向运动和任务顺序，不模拟实体碰撞、轮胎打滑、惯性、移动行人、真实雷达噪声或三维点云重定位。仿真表现不能代替实车避障、制动和重定位验证。

## 11. 单独打开或关闭 RViz

三个启动脚本默认都会打开 RViz，正常使用不需要再启动第二个窗口。若启动时设置了 `SENTRY_USE_RVIZ=False`，可在有图形会话的终端补开窗口。

实车建图或导航的 RViz：

```bash
source /opt/ros/humble/setup.bash
source ~/sentry_nav_071/install/setup.bash
ros2 launch pb2025_nav_bringup rviz_launch.py
```

默认仿真域的 RViz：

```bash
ROS_DOMAIN_ID=71 rviz2 -d \
  "$(ros2 pkg prefix --share pb2025_nav_bringup)/rviz/nav2_map_simulation.rviz"
```

仅关闭当前用户的所有 RViz 窗口：

```bash
pkill -INT -x rviz2
```

关闭 RViz 不等于停止导航或双目标任务。若要让整套会话退出，使用下一节的命令。SSH 下 RViz 未显示时，检查当前终端是否能访问 MiniPC 的图形会话，可先查看 `echo "$DISPLAY"`；不要仅靠启动 RViz 判断导航是否正在运行。

## 12. 停止进程与重新启动

### 12.1 推荐方式

在启动终端按一次 `Ctrl+C`，等待会话退出。建图还要等 PGM/YAML/PCD 保存完成，然后才能重新启动另一模式。双目标任务完成后 Nav2 仍运行，也要退出后再启动新的整套会话。

### 12.2 从另一个终端统一请求退出

查看本项目启动进程：

```bash
pgrep -af 'tools/mapping_session[.]py|ros2 launch pb2025_nav_bringup'
```

一条命令向建图会话管理程序、实车 launch 和仿真 launch 发送正常中断请求：

```bash
pkill -INT -f -- 'tools/mapping_session[.]py|ros2 launch pb2025_nav_bringup (rm_navigation_reality_launch[.]py|map_simulation_launch[.]py)'
```

该命令会影响当前用户下匹配这些入口的所有会话。没有匹配进程时，`pkill` 返回 1；匹配成功只表示信号已发出，还需等待节点关闭和地图保存，再检查进程。它不清理其他项目或其他方式单独启动的节点。

如果只想退出一个会话，先查看实际 PID，再发送信号。下例中的 `12345` 必须替换为上面查到的 PID：

```bash
kill -INT 12345
```

建图会话管理程序也接受 `SIGTERM`，会转换为对子 launch 的正常 `SIGINT` 并等待保存。

### 12.3 会话锁仍被占用

```bash
fuser -v "${SENTRY_MAP_ROOT:-$HOME/sentry_maps}/.navigation.lock"
```

锁随进程持有的文件描述符释放；锁文件留在目录里是正常的。查找并结束占用它的旧会话，不要删除锁文件来绕过并发保护。不同 `SENTRY_MAP_ROOT` 的锁相互独立，也不会保护直接手动运行的 launch。

### 12.4 保存运行日志

需要同时看终端并记录日志时，例如建图：

```bash
mkdir -p "$HOME/sentry_run_logs"
./start_mapping.sh 2>&1 | tee "$HOME/sentry_run_logs/mapping_$(date +%Y%m%d_%H%M%S).log"
```

实车导航或仿真可将上述入口换为对应命令。ROS 自身日志默认还保存在 `~/.ros/log/`。带管道启动时仍在该启动终端按 `Ctrl+C`，等待保存完成。

## 13. 参数与环境变量

### 13.1 启动脚本环境变量

推荐将变量写在单条命令前，这样不会影响其他终端或后续模式。

| 变量 | 默认值 | 用途 |
| --- | --- | --- |
| `SENTRY_ROS_SETUP` | `/opt/ros/humble/setup.bash` | ROS 环境文件 |
| `SENTRY_INSTALL_SETUP` | 项目根目录下的 `install/setup.bash` | 本工作区环境文件 |
| `SENTRY_MAP_ROOT` | `$HOME/sentry_maps` | 保存地图、`latest.json` 和会话锁的位置 |
| `SENTRY_USE_RVIZ` | `True` | 三个脚本是否打开 RViz |
| `SENTRY_ENABLE_MCU_SERIAL` | `True` | 建图/实车导航是否启动串口桥；仿真不使用它 |
| `SENTRY_SERIAL_PORT` | `/dev/serial_port` | 建图/实车导航的串口设备 |
| `SENTRY_AUTO_EXPLORE` | `False` | 建图的探索开关，当前使用方式保持 `False` |
| `SENTRY_ROUTE_CONFIG` | 项目根目录的 `config/two_point_route.yaml` | 实车双目标配置；仿真 `--two-point` 也读取它 |
| `SENTRY_SIM_DOMAIN_ID` | `71` | 仿真 ROS 域，取 0 至 232 且须不同于当前终端实车域 |
| `ROS_DOMAIN_ID` | 未设置时通常为 `0` | 实车 ROS 域；检查命令要与所观察的会话一致 |

若以前在终端导出了自主探索开关，可以明确覆盖：

```bash
SENTRY_AUTO_EXPLORE=False ./start_mapping.sh
```

### 13.2 常用 launch 参数

建图和实车导航入口允许在命令末尾传入未被入口保护的 `name:=value` 参数，例如：

```bash
./start_mapping.sh use_rviz:=False
./start_navigation.sh 1.0 2.0 4.0 2.5 use_composition:=False
./start_navigation.sh 1.0 2.0 4.0 2.5 mcu_serial_baudrate:=115200
```

| 参数 | 当前默认值 | 功能 |
| --- | --- | --- |
| `use_rviz` | `True` | 显示 RViz |
| `use_composition` | `True` | 使用 ROS 组件容器；排查独立节点时可设 `False` |
| `use_respawn` | `False` | 非组件模式下支持的节点是否异常重启 |
| `params_file` | `config/reality/nav2_params.yaml` 的安装路径 | 自定义整套运行参数 |
| `enable_pgm_initial_pose` | `True` | 实车导航启动二维地图初始位姿辅助程序 |
| `enable_auto_explore` | `False` | 建图自主探索开关，当前保持关闭 |
| `mapping_bringup_delay_sec` | `3.0` | 建图处理链在雷达驱动之后的启动延时 |
| `mcu_serial_port` | `/dev/serial_port` | 底盘串口路径 |
| `mcu_serial_baudrate` | `115200` | 底盘串口波特率 |
| `mcu_serial_read_mode` | `line` | 上行串口解析方式，`line` 或 `binary` |
| `mcu_line_numeric_mode` | `command` | 行模式数字消息如何分类，`command`、`status` 或 `both` |
| `mcu_cmd_send_rate_hz` | `50.0` | 下行速度发送频率 |
| `forward_only_enabled` | `False` | 仅前向运动限制；当前全向底盘保持关闭 |

全部 launch 参数及说明可在已编译环境中查看，此命令不启动机器人：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py --show-args
ros2 launch pb2025_nav_bringup map_simulation_launch.py --show-args
```

`start_mapping.sh` 不允许覆盖 `mode`、`slam`、`map`、`prior_pcd_file`；`start_navigation.sh` 还不允许覆盖双目标开关与 `route_x1` 等坐标参数，坐标应使用四个位置参数或配置文件。仿真 `.sh` 只接受 `--two-point` 或四个坐标，以及 `use_rviz:=True/False`，不能把实车参数直接传给它。

## 14. 运行状态检查与故障排查

以下检查在另一个已加载 ROS/工作区环境的终端执行。实车使用相同的 `ROS_DOMAIN_ID`；仿真默认在命令前加 `ROS_DOMAIN_ID=71`。频率监测、持续 echo 和 TF 监测使用检查终端的 `Ctrl+C` 结束，不会停止原启动终端。

### 14.1 节点、雷达和建图数据

```bash
ros2 node list
ros2 topic list -t
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /registered_scan
ros2 topic hz /odometry
ros2 topic info /map --verbose
```

频率命令逐条运行和结束。雷达 IMU 和点云有数据后，检查处理后的点云、里程计、地图是否继续更新；不要只看进程名存在就认定传感器正常。不同 DDS/QoS 设置下频率工具测得的值也可能与实际发布频率不同。

### 14.2 TF、定位和 Nav2

```bash
ros2 run tf2_ros tf2_echo base_footprint front_mid360
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map gimbal_yaw_fake
ros2 topic echo /localization_ready --qos-durability transient_local --once
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
```

`base_footprint` 是机器人在地面的基准坐标系，`front_mid360` 是雷达坐标系，`gimbal_yaw` 表示当前模型中的机器人朝向，`gimbal_yaw_fake` 是导航使用的虚拟朝向参考系，不是一个需要单独“启动”的硬件节点。它们的关系由机器人描述、传感器里程计和速度变换程序共同维护。

建图模式不使用导航重定位的 `localization_ready`；实车双目标导航必须等它有效，Nav2 节点也应进入 `active`。看到 `map` 或 `base_footprint` 暂缺时，先排查雷达/IMU、机器人模型展开和完整 TF 链，不要先改 frame 名称来掩盖缺失链路。

### 14.3 规划路径、速度和串口

```bash
ros2 action list -t
ros2 topic hz /plan
ros2 topic echo /cmd_vel_nav2_result
ros2 topic echo /cmd_vel
ros2 topic echo /mcu/cmd_vel_effective
```

从上到下分别检查导航动作、规划路径、平滑后的导航速度、转换后的底盘速度，以及串口桥补偿/超时处理后准备下发的有效速度。`/mcu/cmd_vel_effective` 有值不代表下位机已经接收或电机已经执行，还需结合串口日志和底盘反馈。

串口占用与内核最近记录：

```bash
fuser -v /dev/serial_port
journalctl -k -n 50 --no-pager
```

设备路径不是默认值时相应替换。关闭了本项目串口桥时，不应期待出现该桥发布的有效速度话题。

### 14.4 常见现象

| 现象 | 检查方法与处理 |
| --- | --- |
| `ROS or workspace setup is missing` | 确认 Humble 已安装、colcon 编译成功，或检查 `SENTRY_ROS_SETUP`、`SENTRY_INSTALL_SETUP` |
| `Permission denied` 无法执行 `.sh` | 在 Ubuntu 项目目录运行 `chmod +x start_mapping.sh start_navigation.sh start_navigation_sim.sh` |
| 建图启动后车不自动走 | 当前就是人工控制建图，自主探索关闭；由实际遥控系统控制移动 |
| 遥控建图被停车速度干扰或串口冲突 | 正常退出后，以 `SENTRY_ENABLE_MCU_SERIAL=False ./start_mapping.sh` 重开 |
| 雷达没有点云或 IMU | 检查供电、有线 IPv4、JSON 主机/雷达地址、UDP 端口和话题发布者 |
| 已有地图但提示无有效地图 | 使用 `map_manifest.py select` 查看原因；检查 PGM/YAML/PCD、记录路径和 `SENTRY_MAP_ROOT` |
| 保存后仍加载旧图 | 新一次保存可能未通过校验；查看退出日志和 `latest.json`，不要只看目录时间 |
| 地图文件修改后拒绝加载 | 核对文件是否仍配套，再用 `record` 重新登记；不要直接篡改校验值 |
| 不带坐标启动导航立即报错 | `config/two_point_route.yaml` 仍是 `null` 或内容非法，填写坐标或传四个数字 |
| 双目标任务一直等待 | 检查点云、`localization_ready`、Nav2 生命周期与新鲜 TF；必要时在 RViz 指定实际初始位姿 |
| 规划了路线但没有运动 | 沿速度话题逐段检查，再检查串口、最低有效速度、底盘状态及定位是否暂停 |
| 点云与地图歪斜或相差 180° | 核对使用的配套地图、雷达外参和初始位姿；在 RViz 确认实际位置与朝向后重新给初始位姿 |
| 仿真看不到机器人或没有 TF | 先用 `2D Pose Estimate` 放置虚拟机器人，确认 RViz 与检查终端都在仿真域 |
| 仿真提示 ROS 域相同 | 检查当前 `ROS_DOMAIN_ID`，让 `SENTRY_SIM_DOMAIN_ID` 选择另一个域 |
| 提示另一个会话正在运行 | 按第 12 节定位并正常结束旧会话，等保存和锁释放，不删除锁文件强行绕过 |
| 只关 RViz 后车还在执行任务 | RViz 是显示和交互客户端，使用第 12 节的整套退出方式 |

## 15. 代码结构与测试

### 15.1 主要文件

| 路径 | 用途 |
| --- | --- |
| `start_mapping.sh` | 人工控制建图入口、会话锁和本次地图路径 |
| `start_navigation.sh` | 自动选图并执行双目标实车导航 |
| `start_navigation_sim.sh` | 自动选图并启动独立域的导航仿真 |
| `config/two_point_route.yaml` | 两个目标的地图坐标 |
| `tools/check_mapping.py` | 建图安装环境预检查 |
| `tools/mapping_session.py` | 管理建图子进程、转发正常停止信号、等待保存 |
| `tools/map_manifest.py` | 校验、登记和选择成套地图 |
| `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/` | 实车与仿真启动文件 |
| `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml` | 实车建图、重定位、导航参数 |
| `src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_overlay.yaml` | 仿真覆盖参数 |
| `src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/` | 地图记录、双目标任务、串口桥、初始位姿等辅助程序 |
| `src/pb2025_sentry_nav/small_gicp_relocalization/` | 修改后的重定位程序 |
| `small_gicp/` | 固定版本的配准库，由重定位包一起编译 |
| `src/nav2_loopback_sim/` | Nav2 运动学仿真器的 Humble 适配版本 |
| `src/pb2025_robot_description/` | 机器人与雷达安装模型 |
| `tests/` | 不依赖运行中的机器人节点的回归检查 |

### 15.2 离线回归测试

在已安装 NumPy、SciPy、OpenCV、PyYAML 的环境中运行：

```bash
cd ~/sentry_nav_071
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

检查范围包括 launch 配置、地图保存与校验、入口参数、双目标状态机、重定位几何和仿真运动/扫描。Linux 下还会运行 Bash 与停止信号相关测试；Windows 下的部分检查会因平台不支持而跳过。

2026-10-31 已在 Ubuntu 22.04、ROS 2 Humble 的 MiniPC 上完成以下验证。当时只连接 MID360 雷达，没有连接底盘串口，自主探索保持关闭。

| 检查项 | 实测结果 |
| --- | --- |
| 依赖与编译 | `rosdep check` 通过，19 个 ROS 包编译成功 |
| 回归测试 | Linux 上 56 项全部通过，包括 Bash 入口、停止信号与仿真启动顺序测试 |
| 雷达建图 | 雷达点云约 20 Hz、IMU 约 125 Hz、配准点云和里程计约 20 Hz，二维地图持续更新 |
| 安装外参 | `base_footprint -> front_mid360` 为 `[-0.07, 0, 0.52] m`，倒装 `roll=3.14 rad` |
| 退出保存 | 成功生成 PGM、YAML、二进制 PCD，校验通过后更新 `latest.json`；该次 PCD 约 102 MB |
| RViz 导航仿真 | 自动加载刚保存的地图，规划、虚拟扫描、全向移动和双目标任务均实际运行 |
| 双目标切换与停车 | 第一点距离约 `0.405 m` 时切换；第二点进入 `0.2 m` 后结束，速度归零后的终点误差约 `0.127 m`，累计虚拟行程约 `5.2 m` |

同日又在新保存的 `356 x 372` 地图上验证了仿真启动顺序：初始位姿设置前，地图服务 Active、规划器和导航器 Unconfigured；设置后自动进入 Active 并完成双目标任务，累计虚拟行程约 `28.8 m`，第一点切换距离约 `0.470 m`，停车后的终点误差约 `0.145 m`。新增 4 项启动状态检查，覆盖位姿未设置、TF 缺失或过期、重复启动请求及启动失败。

本次为静止雷达生成的局部验证地图，只能在已知可通行区域测试，不代表完整现场地图。停止建图时还观察到 Livox、RViz 的非零退出日志和组合容器退出超时回收，地图与 PCD 保存完成且无残留建图进程；第三方进程的退出行为仍需继续排查。实车行驶、避障、制动和三维点云重定位尚未在本次硬件接线条件下验收。

### 15.3 范围、固定依赖与第三方许可

项目保留 Livox、Point-LIO、SLAM Toolbox、Nav2、全向控制、串口速度桥和修改后的重定位；云端、MQTT、摄像头、图像识别、视频流和 UWB 跟随不在这个比赛用工作区内。探索源码作为依赖保留，建图入口与 launch 的默认开关均关闭。

`small_gicp/` 固定于原 MiniPC 使用的版本 `78f2e7a221720625eb95271ad9da21a04fb77f86`。仿真器来自 Nav2 `1.3.14`，上游提交 `58e4019eb149ed4387938d9cd13f38659b86f754`，来源与本地适配见 [仿真包说明](src/nav2_loopback_sim/README.md)。

各第三方代码和资源的版权、许可证保留在原目录，包括 [导航代码许可](src/pb2025_sentry_nav/LICENSE)、[探索代码许可](src/m-explore/LICENSE)、[仿真代码许可](src/nav2_loopback_sim/LICENSE) 和 [small_gicp 许可](small_gicp/LICENSE)。

首次上车仍需逐项验证速度方向、串口超时停车、外参、足迹、制动距离、避障和重定位。软件检查或仿真通过不等于比赛场地的实车验证，实际底盘调试需保留独立可用的急停。

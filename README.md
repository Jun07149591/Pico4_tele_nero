# pico4_tele_nero

PICO 4 Ultra + XRoboToolkit 驱动 Nero 人形双臂的本地遥操与示范数据采集项目。
项目支持先控制右臂的单臂模式，也支持右臂 `can0` 与左臂 `can1` 同时控制；输入、逆运动学、CAN 输出、相机采集和 LeRobot/OpenPI 导出都在同一个目录中。

## 功能

- PICO 4 Ultra 控制器和头显追踪接收，XRoboToolkit PC Service 本地转发。
- 右手柄控制右臂，左手柄控制左臂；单臂启动不依赖左臂。
- 侧装人形双臂坐标映射：头显视野前方、上方、右方对应机器人身体的前方、上方、右方，位移 1:1。
- Placo 逆运动学、相对位姿跟随、夹爪控制、回位姿态和输入中断恢复。
- D455 顶部相机和腕部相机同步采集，网页录制、回放、质检和导出。
- 录制由用户手动结束，不限固定时长；B/Y 回位继续采集，数据暂时缺失时保留片段并记录缺帧。
- 导出为 LeRobot v2.1 视频数据集：每个 episode 一个 Parquet，各相机一个 H.264 MP4，只使用 `videos/`，不生成重复的 `clips/`。

## 目录

```text
pico4_tele_nero/
  teleop/                  # 遥操 Python 包、机器人配置、URDF、单元测试
  data_collection/         # 采集网页、HDF5 存储、视频导出、OpenPI 适配
  vendor/pyAgxArm/         # Agilex SDK Python 源码（LGPL）
  vendor/xr-releases.json  # XRoboToolkit 固定版本下载地址和 SHA-256
  scripts/                 # 安装、启动、CAN、相机、测试入口
  docs/                    # 中文部署和数据格式说明
```

真实录制、导出、虚拟环境和 XRoboToolkit 二进制默认写入 `.gitignore` 中的路径，不会被上传到 GitHub。

## 系统要求

当前安装脚本针对 Ubuntu 22.04/24.04 x86_64。需要：

- Conda/Miniforge、Python 3.11/3.12、sudo 权限；
- 两个 1 Mbit/s USB-CAN 适配器，分别连接 `can0`/`can1`；
- PICO 4 Ultra、两个控制器、XRoboToolkit PICO APK；
- Intel RealSense D455（顶部）和 D405（腕部）；
- 若训练或运行策略，需要另行准备 OpenPI 环境。

## 首次安装

先按 [Miniforge 官方安装说明](https://github.com/conda-forge/miniforge#install) 安装 Conda，并确认 `conda --version` 可用。系统依赖：

```bash
sudo apt update
sudo apt install -y git curl python3 iproute2 can-utils usbutils adb \
  libgl1 libglib2.0-0 libusb-1.0-0 libxkbcommon0 libxcb-cursor0
```

RealSense 的 Python wheel 不会安装系统设备权限规则。新电脑还需按 [librealsense Linux 安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md) 安装设备规则，安装后重新插拔相机。普通用户运行采集程序，不要使用 `sudo` 启动 Python。

```bash
cd pico4_tele_nero
bash scripts/setup.sh
bash scripts/check.sh
```

**首次使用必须完成安装后再启动遥操。** GitHub 源码和源码 ZIP 不包含 XRoboToolkit 的二进制运行文件；即使这台电脑已经有 Python 环境，换到新项目目录后也要确认该目录的 XR 文件已安装。

安装脚本在用户缓存目录创建三个环境（默认 `~/.cache/pico4_tele_nero/envs`），不依赖工程所在路径，也支持路径包含空格。首次安装需要能访问 PyPI、GitHub 和 OpenPI/LeRobot 的固定 revision。可用环境变量修改位置：

```bash
export PICO_ENV_ROOT="$HOME/pico4_nero_envs"
bash scripts/setup.sh
```

默认 `setup.sh` 会安装遥操和采集环境，并下载、校验和解压 XRoboToolkit PC Service、C SDK 和 PICO APK。等脚本最后输出 `Setup complete. Environments: ...`，再运行 `check.sh`；仅看到 `Teleop dependencies ready` 或 `Capture/export dependencies ready`，不表示完整安装已经结束。

安装后应有以下文件，它们保存在本机，不提交到 GitHub：

```text
.runtime/
  roboticsservice/RoboticsServiceProcess
  roboticsservice/SDK/x64/libPXREARobotSDK.so
  XRoboToolkit-PICO-1.1.1-SDK29.apk
```

PC 服务只解压到项目目录，安装脚本不会自动启动服务或配置开机自启。APK 下载到电脑后，还需按 [中文使用手册](docs/USAGE_ZH.md#1-首次准备) 安装到 PICO。

### 已有 Python 环境，补装 XR 文件

如果使用过 `setup.sh --skip-xr`、安装中途失败，或新目录只复用了 Python 环境，启动时可能提示 `XR service missing`，预检也可能提示 `libPXREARobotSDK.so: cannot open shared object file`。在当前项目根目录执行：

```bash
bash scripts/fetch_xr.sh
bash scripts/check.sh
bash scripts/start_pico_service.sh
```

`fetch_xr.sh` 只下载、校验和安装 XR 文件，不重装 Python 环境。默认离线检查 `check.sh` 应显示 `"technical_checks_passed": true` 和 `"errors": []`；这表示依赖可用，不代表 PICO 追踪和真机连接已经就绪。最后一条命令启动 PC 服务，正常启动后保持该终端运行，再另开终端启动遥操。

`setup.sh --teleop-only` 包含遥操环境和 XR 文件；`--data-only` 只安装采集环境，不构成完整遥操安装；`--skip-xr` 明确跳过 XR 文件。首次搭建完整流程请使用不带这些选项的 `bash scripts/setup.sh`。

如果现场已经有 SDK 文件，可设置 `PICO_XR_SDK` 指向 `libPXREARobotSDK.so`，设置 `NERO_AGX_SDK_ROOT` 指向 `pyAgxArm` 目录。`PICO_XR_SDK` 只改变 Python 加载的库路径，不会安装或启动 PC 服务；本项目的 `start_pico_service.sh` 仍需要上述 `.runtime/roboticsservice` 文件。

## 配置设备

### 相机

先查看相机：

```bash
bash scripts/data.sh cameras
```

按型号自动绑定 D455 为顶部、D405 为右腕：

```bash
bash scripts/configure_cameras.sh --auto
```

或者显式指定 `cameras` 命令列出的序列号（替换下方占位值）：

```bash
bash scripts/configure_cameras.sh \
  --front YOUR_D455_SERIAL \
  --right-wrist YOUR_D405_SERIAL
```

它会生成 `data_collection/config/capture.local.json`。采集默认 30 Hz，可在网页中修改为不超过相机帧率的整数频率。

### CAN

插拔适配器或重启后执行：

```bash
bash scripts/configure_can.sh --dual
```

单臂只需要 `can0`：`bash scripts/configure_can.sh --single`。两条总线应显示 `ERROR-ACTIVE` 和 `1000000` bit/s。此脚本只配置总线，不发送运动目标，也不会重置正在运行的总线。接口已经启用但波特率不正确时，先停止控制程序，再按 [使用手册](docs/USAGE_ZH.md) 的故障处理步骤重新配置。

## 启动遥操

在 PICO 上安装并打开 `.runtime/XRoboToolkit-PICO-1.1.1-SDK29.apk`，连接运行 PC Service 的电脑并开启追踪发送。电脑端启动服务：

```bash
bash scripts/start_pico_service.sh
```

另开终端启动右臂单臂模式：

```bash
bash scripts/start_teleop.sh --single --confirm-clearance --enable-joints --duration 3600
```

双臂模式：

```bash
bash scripts/start_teleop.sh --dual --confirm-clearance --enable-joints --duration 3600
```

启动前必须确认机械零位、侧装姿态和运动范围。程序不会写零位或修改官方关节限位。先松开 Grip/Trigger，等待输入进入中立状态和所选手臂回位完成，再按住对应手柄 Grip 开始跟随。松开 Grip 会保持该臂目标；重新握持会重新建立锚点。摇杆向上闭合夹爪、向下打开，回中停止改变开合目标，不需要同时按 Grip。右手柄 B、左手柄的 secondary 按键（PICO 通常标为 Y）分别回到对应手臂的保存姿态。

PICO 页面显示 TCP 连接失败时，检查 PC Service 是否监听、头显与电脑是否同一局域网、连接地址是否为电脑的局域网 IP、XRoboToolkit 是否在前台并持续发送。

## 启动采集网页

采集需要遥操程序通过本机 socket 发布遥测。启动采集服务：

```bash
mkdir -p data/right_arm
bash scripts/data.sh serve --mode single --root data/right_arm --port 8765
```

浏览器打开 <http://127.0.0.1:8765>。在“采集”页填写任务，点击开始和成功/失败；在“片段与质检”页回放并标记 PASS；在“导出”页勾选同一频率的片段，填写 `namespace/dataset` 名称后导出。双臂采集使用 `--mode dual` 和独立的数据根目录。

导出目录示例：

```text
data/right_arm/exports/nero_pick_place_20260914T120000_000000/local/nero_pick_place/
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.front/episode_000000.mp4
  meta/nero_provenance.json
  meta/nero_openpi.json
```

网页会显示每个原始 HDF5 与 Parquet/MP4 的对应关系。官方 episode 索引从 `000000` 开始，两个片段应得到两个 Parquet 和每路相机各两个 MP4。将 `meta/nero_openpi.json` 传给 OpenPI 适配入口即可。

完整操作、按钮含义、保存/导出区别和常见问题见 [中文使用手册](docs/USAGE_ZH.md)。无需连接 PICO、机械臂或相机，可以先运行合成数据演示：

```bash
bash scripts/data.sh demo --root data/demo --port 8766
```

演示网页为 <http://127.0.0.1:8766>，会显示模拟数据标识，不控制真实硬件。



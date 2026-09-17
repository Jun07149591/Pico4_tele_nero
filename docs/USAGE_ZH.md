# 中文使用手册

适用场景：PICO 4 Ultra 通过 XRoboToolkit 控制侧装 Nero 七自由度机械臂，同时采集 RGB、关节状态和动作目标。右臂为 `can0`，左臂为 `can1`，各自对应同侧手柄。本文所有命令均在仓库根目录运行。

## 1. 首次准备

### 安装完整运行环境

先按 [README](../README.md) 安装系统包、Conda、RealSense 设备权限规则，再在当前项目根目录运行：

```bash
bash scripts/setup.sh
bash scripts/check.sh
```

**克隆源码不等于完成安装。** 源码仓库不包含虚拟环境、XRoboToolkit PC Service、C SDK 和 APK。默认 `setup.sh` 会创建 Python 环境，并从官方固定版本下载、校验和解压 XR 文件。即使电脑上已经有遥操 Python 环境，新项目目录也需要自己的 XR 运行文件。

安装脚本最后应输出 `Setup complete. Environments: ...`。只看到 `Teleop dependencies ready` 或 `Capture/export dependencies ready` 时，后续 XR 安装可能还没完成。若脚本报错，先处理该错误；不要直接跳到启动遥操。

完整安装后，项目根目录应包含：

```text
.runtime/
  roboticsservice/RoboticsServiceProcess
  roboticsservice/SDK/x64/libPXREARobotSDK.so
  XRoboToolkit-PICO-1.1.1-SDK29.apk
```

`check.sh` 默认只检查依赖、模型、Placo 和 SDK 加载，不连接真机或发送运动指令。检查结果应为 `"technical_checks_passed": true`、`"errors": []`。硬件和追踪是否就绪，在每日启动时另行检查。

安装选项的区别：

| 命令 | 安装内容 |
| --- | --- |
| `bash scripts/setup.sh` | 完整安装：遥操环境、采集环境、XR PC 服务、SDK 和 APK |
| `bash scripts/setup.sh --teleop-only` | 遥操环境及 XR 文件，不安装采集环境 |
| `bash scripts/setup.sh --data-only` | 只安装采集环境，不安装遥操环境和 XR 文件 |
| `bash scripts/setup.sh --skip-xr` | 安装 Python 环境，跳过 XR 文件，需之后补装 |

### 补装缺失的 XR 文件

如果启动提示 `XR service missing. Run bash scripts/fetch_xr.sh.`，或预检报告中出现 `libPXREARobotSDK.so: cannot open shared object file`，先运行：

```bash
bash scripts/fetch_xr.sh
bash scripts/check.sh
```

这两步用于已有 Python 环境的情况，不需要重新安装整套环境。`fetch_xr.sh` 需要联网访问 GitHub，默认把安装包缓存到 `.downloads/`，通过 SHA-256 校验后安装到 `.runtime/`。这些目录被 Git 忽略；在本机正常安装后，无需每天重复下载。

若出现 `Python environment missing`，则需要执行完整的 `bash scripts/setup.sh`。自定义 `PICO_XR_SDK` 只改变 SDK 库路径，不能代替 PC 服务的安装。

### 绑定相机和安装 PICO 应用

```bash
bash scripts/data.sh cameras
bash scripts/configure_cameras.sh --auto
```

`--auto` 只在恰好连接一台 D455 和一台 D405 时自动分配角色。两个同型号腕部相机必须用 `--right-wrist SERIAL --left-wrist SERIAL` 显式区分。相机配置写入本机 `capture.local.json`。配置后重启采集服务才会生效。

PICO 开启开发者模式和 USB 调试，插入 USB 数据线并在头显中授权。安装 APK：

```bash
adb devices
adb install -r .runtime/XRoboToolkit-PICO-1.1.1-SDK29.apk
```

`adb devices` 应显示 `device`；`unauthorized` 表示尚未授权。安装 APK 不会自动连接 PC 或开启追踪。

## 2. 每天开机后

1. 检查机械臂安装、工具和线缆，确认官方控制器仍使用侧装配置和正确机械零位。
2. 确认右臂适配器是 `can0`，双臂模式还需确认左臂是 `can1`。
3. 开机或插拔 USB-CAN 后配置总线，单臂用 `--single`，双臂用 `--dual`。

```bash
ip -brief link
bash scripts/configure_can.sh --single
```

4. 另开终端启动 XRoboToolkit PC Service，并保持运行。

```bash
bash scripts/start_pico_service.sh
```

安装程序不会自动启动 PC 服务，也不配置开机自启。正常启动后该终端持续运行；若提示 `XR service missing`，先按上面的补装步骤处理，再启动服务。提示 `XRoboToolkit PC service is already running.` 表示已有服务进程，无需重复启动。

5. 戴上 PICO，打开 XRoboToolkit，连接电脑并启动控制器追踪发送。头显连接电脑的局域网 IP；`127.0.0.1:60061` 是电脑内 Python 到 PC Service 的地址。
6. 松开所使用手柄的 Grip、Trigger、功能键，并让摇杆回中。

仅插 USB 不代表追踪链路已经连通。USB 可用于安装、供电、ADB 或已配置的网络转发，数据是否可用以 XRoboToolkit 持续发送和遥操预检为准。

首次连接或排查启动失败时，可在另一个终端运行只读连接检查：

```bash
bash scripts/check.sh --live --duration 5
```

该命令默认检查右臂 CAN 反馈和 PICO 中立输入，不发送运动目标。双臂启动入口还会分别检查左右两臂。

## 3. 启动遥操

单臂：

```bash
bash scripts/start_teleop.sh --single --confirm-clearance --enable-joints --duration 3600
```

双臂：

```bash
bash scripts/start_teleop.sh --dual --confirm-clearance --enable-joints --duration 3600
```

`--confirm-clearance` 明确授权真机输出；`--enable-joints` 允许接管时使能关节；`--duration 3600` 表示最多运行一小时。默认启动模式为单臂。单臂完全不依赖左臂或左手柄反馈。

程序先检查输入、模型、CAN 和反馈，再接管和回位。保持手柄释放，等待所选机械臂的 `home_return_complete` 和 `teleop_ready`，之后再按 Grip。启动回位途中按 Grip/Trigger 会取消回位，并且不会宣布遥操就绪。

### 控制器

| 控件 | 功能 |
| --- | --- |
| 按住对应 Grip | 启用该臂位姿跟随 |
| 松开 Grip | 保持最后目标，停止推进新目标 |
| 重新按 Grip | 用最新实际反馈重新建立相对锚点 |
| 摇杆上推 | 闭合对应夹爪；推得越多开合目标变化越快 |
| 摇杆下推 | 打开对应夹爪 |
| 摇杆回中 | 保持夹爪开合目标 |
| 右 B / 左 secondary（通常标为 Y） | 返回该臂保存的回位姿态 |
| 右 A / 左 primary（通常标为 X） | 请求重建参考，释放后重新握持 |
| Trigger | 不控制夹爪，参与中立检查和回位取消 |
| Menu | 使输入失效，释放并重新建立中立状态 |
| 终端 Ctrl-C | 退出遥操，停止推进目标 |

摇杆无需与 Grip 同时按住，但要求追踪有效、已完成中立授权并且不在回位。B/Y 回位时先松开 Grip/Trigger；按键只控制对应手臂，不会自动同时让两臂回位。

摇杆推到底时夹爪开合目标速度为 60 mm/s。回位目标上限为关节 60 deg/s、末端平移 150 mm/s、末端旋转 40 deg/s，具体实测速度取决于姿态、负载和设备响应。配置位置见 [配置说明](CONFIGURATION.md)。

回位姿态保存在 `teleop/config/home_poses.json`，来自参考设备的实际姿态。它不是七关节全零位。换安装方式、TCP、机械零位或机器人后，不能直接把旧回位姿态视为已验证的新设备配置。

当前右臂初始姿态更新于 2026-09-15 16:20，通过 `can0` 只读获取：七关节角度为 `[-44.410, 84.470, -87.622, -56.013, 0.415, -3.385, -66.100]` 度，同时保存了同一模型下的 TCP 位置和旋转矩阵。启动回位与右 B 键共用此目标，左臂仍使用原姿态。配置在下次启动遥操时加载。

## 4. 采集数据

在另一个终端启动采集网页，保持遥操运行：

```bash
bash scripts/data.sh serve --mode single --port 8765
```

双臂使用独立目录：

```bash
bash scripts/data.sh serve --mode dual --port 8765
```

模式必须与遥操一致；两种服务不能同时使用同一个端口或同一个录制目录。顶层遥操入口自动发送遥测到 `/tmp/nero_pico_data_<UID>.sock`。同一用户可以先开网页或先开遥操，连接就绪后即可开始采集。

### 数据保存位置

`serve` 和终端采集命令 `record` 不传 `--root` 时，根据实际采集模式自动选择项目内目录；模式来自 `--mode`，未指定时使用相机配置中的 `mode`。

| 模式 | 默认目录 |
| --- | --- |
| 单臂 | `<项目目录>/data/right_arm` |
| 双臂 | `<项目目录>/data/dual_arm` |
| 模拟演示 `demo` | `<项目目录>/data/demo` |

默认目录相对于脚本所属的项目解析，不依赖终端当前工作目录，也不依赖原电脑用户名。启动时自动创建目录并打印绝对路径，网页“导出”页下方也显示原始数据目录。

需要独立任务目录或外部硬盘时，可显式传入 `--root /path/to/dataset`。显式相对路径仍按终端当前工作目录解析，例如在项目根目录运行 `--root data/pick_place`。`review`、`validate`、`export` 操作现有数据，仍要求明确传入 `--root`。

旧目录中的数据不会自动迁移，也不会合并到新的默认目录。继续采集旧数据时传入旧目录；移植已有数据时，在采集和导出均停止后，将整个数据根目录复制到新机器，再指定复制后的目录。原始片段、质检记录、manifest 和编号记录应一起保留。`data/` 被 Git 忽略，克隆 GitHub 项目不包含已采集的数据。

### 网页操作

浏览器打开 <http://127.0.0.1:8765>：

1. 查看相机画面和“已就绪”状态，填写描述动作目标的任务指令。
2. 默认 30 Hz。需要修改时，在待机状态下更改频率并应用；录制期间不允许修改。
3. 点击开始，使用手柄完成任务。
4. 手动点击成功、失败或丢弃结束本段。每段生成一个 HDF5。
5. 进入片段与质检页，回放画面和 state/action，合格片段标记 PASS。
6. 进入导出页，勾选需要的 success + PASS 片段，填写 `local/你的数据集名`，等待导出完成。

### 录制快捷键

在浏览器的“采集”页使用，网页需要处于前台。填写完任务后点击页面空白处，让输入框失去焦点：

| 按键 | 操作 |
| --- | --- |
| 空格，当前未录制 | 开始录制，需要任务已填写、采集频率已应用且数据已就绪 |
| 空格，当前正在录制 | 成功保存本段 |
| L，当前正在录制 | 失败保存本段，大小写均可 |

长按不会连续触发，保存请求处理中也不会重复发送。在任务、频率、备注等输入框中编辑文字，以及“片段与质检”“导出”页内，不触发录制快捷键。原有鼠标按钮仍可使用。

录制没有固定 15 秒或 180 秒时长，持续到手动点击成功、失败或丢弃。成功/失败保存至少需要 20 帧，过短片段可丢弃。旧配置中的 `max_episode_seconds` 已停用，即使仍填写数字也不会自动结束。

录制中可以按 B/Y 回位，回位过程的图像、实际关节状态及已发送目标继续写入同一片段；回位完成不会停止录制。短暂反馈延迟最多重试 250 ms，若仍缺少有效相机或遥测数据，网页显示“录制中 · 等待数据”和缺失帧数，保留当前片段，数据恢复后继续。等待期间也可以手动保存或丢弃。

真实缺帧会在 HDF5 中留下 `capture_gaps` 记录，不能补造为连续数据。有缺帧的片段可以保存、回放和检查，但不会通过训练数据质检；重新采集完整示范后再导出。失败、未检查、中断片段也不会自动进入训练集。一次导出必须使用同一采样频率。

手动结束要求采集服务持续运行且磁盘可写。退出采集服务会尽力保存为 `interrupted`；断电、磁盘故障无法保证完整保存。正常操作应先在网页结束本段，再退出服务。网页录制计时为实际经过时间，保存片段时长为有效帧数除以采样频率，有缺帧时两者可能不同。

### 删除片段

在“片段与质检”页勾选一个或多个片段，点击上方垃圾桶图标，核对确认框内的文件名后点击删除。“全选当前列表”只选择当前筛选条件下的片段；切换筛选条件会保留之前的勾选，确认框列出本次删除的全部片段。

删除会移除原始 HDF5 片段及对应质检记录，释放这些文件占用的空间；已经导出的 Parquet/MP4 数据集是独立副本，继续保留。录制、保存或导出期间不能删除。删除后的原始 episode 编号不会复用，例如删除 `episode_17.h5` 后，新片段仍从 `episode_18.h5` 开始，其他片段不重新编号。原目录和离线质检模式同样支持此操作。

### 保存与导出的区别

- 保存：`<root>/episodes/episode_01.h5`，保留原始帧、采样时间、关节、夹爪和诊断，供回放与重新导出。
- 导出：`<root>/exports/<数据集名>_<时间戳>/<namespace>/<dataset>/`，生成供 LeRobot/OpenPI 使用的数据。
- 导出编号从 `episode_000000` 开始，按所选原始采集顺序连续排列。
- 两个片段、两个相机视角会生成两个 Parquet、四个 MP4。
- 正式视频统一在 `videos/`，不会额外建立 `clips/`。
- 数据集名称不会改变任务指令。名称标识数据集，任务指令作为每段训练 prompt。

只需要回放或导出旧数据时，可以不启动硬件：

```bash
bash scripts/data.sh review --root data/right_arm --port 8765
```

## 5. 正常结束

先结束并保存正在录制的片段，等待保存/导出完成；再松开手柄，Ctrl-C 退出遥操，最后关闭采集网页服务和 PC Service。退出遥操不等于切断电机电源，也不等于已确认物理停止；机械臂上下电遵循设备本身操作规程。

## 6. 常见问题

| 现象 | 检查与处理 |
| --- | --- |
| `XR service missing` | 当前项目没有安装 PC 服务；运行 `bash scripts/fetch_xr.sh`、`bash scripts/check.sh`，再启动 `bash scripts/start_pico_service.sh` |
| `libPXREARobotSDK.so: cannot open shared object file` | 若报告中的缺失路径位于 `.runtime/`，按上面的 XR 补装步骤处理；使用自定义 `PICO_XR_SDK` 时，检查它是否仍指向有效文件 |
| `Python environment missing` | 先运行 `bash scripts/setup.sh`；只运行 `fetch_xr.sh` 不会创建 Python 环境 |
| PICO 显示 connect error | 检查 PC Service、局域网地址、网络连通和应用前台发送；USB 插着不代表追踪已发送 |
| `PICO must send fresh controller input` | 戴好头显，确认所用控制器被追踪，松开 Grip/Trigger 和功能键，再启动 |
| `tracking_stale` / 输入超时 | 查看前面的追踪中断事件；恢复发送后释放并重新握持，严重错误退出后需重新启动 |
| `dataset already in use (PID ...)` | 使用已有网页或在原终端 Ctrl-C 正常退出；不要删除运行中采集器的 `.writer.lock` |
| `[Errno 98] Address already in use` | 当前端口已有服务，复用它或使用另一个 `--port` |
| 等待相机画面 | 用 `data.sh cameras` 检查设备和序列号，检查 USB 3、设备权限、是否被其他程序独占 |
| 相机正常但不能录制 | 查看网页未就绪原因：常见为遥操尚未就绪、模式不一致、夹爪或反馈缺失；已就绪后的 B/Y 回位不阻止采集 |
| 录制中显示等待数据 | 片段仍在录制，恢复相机/遥测后继续；可手动保存或丢弃，有真实缺帧的片段不能通过质检 |
| 只能导出一个片段 | 检查实际勾选数、success/PASS 标记和频率，查看本次导出结果的逐段清单 |
| `CANCELLED` / `uninitialize sdk` | 通常是 SDK 退出日志；真正原因在前面的 `teleop_complete.reason/error` 或预检报告中 |

若 CAN 已启用但波特率不对，在停止遥操后执行（只针对需修正的接口）：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

更换相机、模式或图像尺寸时使用新的采集根目录，避免混淆已有 manifest。只改采集频率可以继续使用原目录，每段保留原频率，导出时按频率分别选择。

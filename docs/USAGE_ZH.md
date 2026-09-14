# 中文使用手册

适用场景：PICO 4 Ultra 通过 XRoboToolkit 控制侧装 Nero 七自由度机械臂，同时采集 RGB、关节状态和动作目标。右臂为 `can0`，左臂为 `can1`，各自对应同侧手柄。本文所有命令均在仓库根目录运行。

## 1. 首次准备

先按 [README](../README.md) 安装系统包、Conda、RealSense 设备权限规则，再运行：

```bash
bash scripts/setup.sh
bash scripts/data.sh cameras
bash scripts/configure_cameras.sh --auto
bash scripts/check.sh
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

5. 戴上 PICO，打开 XRoboToolkit，连接电脑并启动控制器追踪发送。头显连接电脑的局域网 IP；`127.0.0.1:60061` 是电脑内 Python 到 PC Service 的地址。
6. 松开所使用手柄的 Grip、Trigger、功能键，并让摇杆回中。

仅插 USB 不代表追踪链路已经连通。USB 可用于安装、供电、ADB 或已配置的网络转发，数据是否可用以 XRoboToolkit 持续发送和遥操预检为准。

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

回位姿态保存在 `teleop/config/home_poses.json`，来自参考设备的实际姿态。它不是七关节全零位。换安装方式、TCP、机械零位或机器人后，不能直接把旧回位姿态视为已验证的新设备配置。

## 4. 采集数据

在另一个终端启动采集网页，保持遥操运行：

```bash
bash scripts/data.sh serve --mode single --root data/right_arm --port 8765
```

双臂使用独立目录：

```bash
bash scripts/data.sh serve --mode dual --root data/dual_arm --port 8765
```

模式必须与遥操一致；两种服务不能同时使用同一个端口或同一个录制目录。顶层遥操入口自动发送遥测到 `/tmp/nero_pico_data_<UID>.sock`。同一用户可以先开网页或先开遥操，连接就绪后即可开始采集。

浏览器打开 <http://127.0.0.1:8765>：

1. 查看相机画面和“已就绪”状态，填写描述动作目标的任务指令。
2. 默认 30 Hz。需要修改时，在待机状态下更改频率并应用；录制期间不允许修改。
3. 点击开始，使用手柄完成任务。
4. 点击成功、失败或丢弃结束本段。每段生成一个 HDF5，异常中断会保留记录供检查。
5. 进入片段与质检页，回放画面和 state/action，合格片段标记 PASS。
6. 进入导出页，勾选需要的 success + PASS 片段，填写 `local/你的数据集名`，等待导出完成。

当前默认单段最长 180 秒，至少 20 帧；这些参数在 `capture.local.json` 中修改。失败、未检查、中断片段不会自动进入训练集。一次导出必须使用同一采样频率。

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
| PICO 显示 connect error | 检查 PC Service、局域网地址、网络连通和应用前台发送；USB 插着不代表追踪已发送 |
| `PICO must send fresh controller input` | 戴好头显，确认所用控制器被追踪，松开 Grip/Trigger 和功能键，再启动 |
| `tracking_stale` / 输入超时 | 查看前面的追踪中断事件；恢复发送后释放并重新握持，严重错误退出后需重新启动 |
| `dataset already in use (PID ...)` | 使用已有网页或在原终端 Ctrl-C 正常退出；不要删除运行中采集器的 `.writer.lock` |
| `[Errno 98] Address already in use` | 当前端口已有服务，复用它或使用另一个 `--port` |
| 等待相机画面 | 用 `data.sh cameras` 检查设备和序列号，检查 USB 3、设备权限、是否被其他程序独占 |
| 相机正常但不能录制 | 查看网页未就绪原因：常见为遥操未启动、模式不一致、正在回位、夹爪或反馈缺失 |
| 只能导出一个片段 | 检查实际勾选数、success/PASS 标记和频率，查看本次导出结果的逐段清单 |
| `CANCELLED` / `uninitialize sdk` | 通常是 SDK 退出日志；真正原因在前面的 `teleop_complete.reason/error` 或预检报告中 |

若 CAN 已启用但波特率不对，在停止遥操后执行（只针对需修正的接口）：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

更换相机、模式或图像尺寸时使用新的采集根目录，避免混淆已有 manifest。只改采集频率可以继续使用原目录，每段保留原频率，导出时按频率分别选择。

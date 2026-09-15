# 配置说明

配置文件都使用相对于自身文件的路径，复制工程或把工程放在包含空格的目录中不会改变路径解析。

## 遥操

- `teleop/config/deployment.json`：单臂/双臂默认选择、速度、1:1 位移比例和 `move_js` 输出模式。
- `teleop/config/nero_humanoid_config.json`：头显/手柄到人形身体坐标的方向矩阵、Placo 权重、实时频率和物理模型引用。
- `teleop/config/teleop_config.json`：左右臂 CAN 通道、夹爪、官方限位来源和控制器手别。
- `teleop/config/humanoid_frames.json`：左右肩安装矩阵。它是人形侧装参考模型，不能替代现场安全检查。
- `teleop/config/home_poses.json`：启动和 B 键回位姿态。它不是机械零位。

`can0` 固定为右臂，`can1` 固定为左臂；适配器顺序改变时必须重新确认物理连接，不要只依赖 Linux 接口名称。每台现场机械臂都应重新确认零位和 USB-CAN 序列号。

当前保留的控制参数为 1:1、75 Hz 目标循环、400 mm/s 平移、120 deg/s 关节、60 deg/s 姿态变化；这些是主机目标限制，不代表保证达到的实测速度或机械臂官方速度极限。`session_limits=false` 和连续 `move_js` 模式保留当前已调好的行为，官方关节模型边界及输入有效性检查仍生效。

夹爪和回位使用独立速度配置，修改后重启遥操生效：

| 配置文件 | 字段 | 默认值 |
| --- | --- | --- |
| `nero_humanoid_config.json` | `gripper_speed_m_s` | `0.06`，摇杆推到底时夹爪开合目标变化 60 mm/s |
| `home_poses.json` | `joint_speed_deg_s` | `60.0`，回位关节速度上限 |
| `home_poses.json` | `tcp_speed_m_s` | `0.15`，回位末端平移速度上限 150 mm/s |
| `home_poses.json` | `angular_speed_deg_s` | `40.0`，回位末端旋转速度上限 |

回位每一步同时满足上述三个速度上限。摇杆推幅越小，夹爪开合越慢；夹爪力度与开合方向不由速度字段改变。

`physical_zero_alignment_verified=true` 保存的是原参考设备已确认的状态；更换机器人后应先置为 `false`，按官方流程检查后再确认。打包没有执行零位写入。
`usb_adapter_serial` 是可选的适配器身份校验，默认不绑定原电脑的设备号。现场可在确认左右物理对应后填写，避免插拔后接口顺序变化。

回位配置包含关节角及 FK 计算的 TCP 位置/旋转矩阵，程序会检查它们是否一致。修改关节角后需按同一 URDF 和 TCP 重新计算这两项，不能只改关节角或把字段误当作机械零位。调整 `on_startup=false` 可关闭自动启动回位，B/Y 保存姿态回位仍保留。

`nero_relative_config.json`、`deployment_legacy.json` 为历史行为的离线回归保留；日常使用统一入口选中的 `nero_humanoid_config.json`。

## 相机

`data_collection/config/capture.json` 是模板，序列号为 `SET_*`。运行 `scripts/configure_cameras.sh` 后生成被 `.gitignore` 忽略的 `capture.local.json`。它只保存设备序列号、分辨率和相机帧率，不包含录制数据。

## 环境变量

| 变量 | 用途 |
| --- | --- |
| `PICO_ENV_ROOT` | Conda/venv 环境目录，默认 `~/.cache/pico4_tele_nero/envs` |
| `PICO_XR_SDK` | 自定义 `libPXREARobotSDK.so` 路径 |
| `NERO_AGX_SDK_ROOT` | 自定义 `pyAgxArm` 路径 |
| `NERO_PLACO_PYTHON` | Placo worker Python 路径 |
| `PICO_NERO_PYTHON` | 覆盖遥操解释器，仅复用已安装的兼容环境时使用 |
| `NERO_DATA_PYTHON` | 覆盖采集解释器，仅复用已安装的兼容环境时使用 |
| `OPENPI_PROJECT_DIR` | 外部 OpenPI checkout |
| `OPENPI_PYTHON` | OpenPI 虚拟环境 Python |

不要把真实设备序列号、导出数据或运行日志提交到公共仓库。

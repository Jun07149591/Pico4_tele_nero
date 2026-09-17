# 第三方声明

## pyAgxArm

`vendor/pyAgxArm` 来自 [agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)，源码快照 revision `a13cd89fe17347dcced1f8c1d26ab2ed0c8a0d1b`。Python 源码与上游本地 checkout 一致，按 LGPL-3.0-only 发布，许可证随目录提供。保留完整包是因为工厂和协议注册会导入共享驱动模块。

## EVA

源项目 [Noietch/EVA-CLIENT](https://github.com/Noietch/EVA-CLIENT)，revision `ced18258151813d50e6264c8a376e386ca85b99e`。
`data_collection/nero_pico_data/eva_alignment.py` 改编自 `src/core/recorder/collection_alignment.py` 的邻近采样、线性插值、离散保持和时间统计方法，加入数组接口、覆盖范围检查和因果动作保持。该上游文件无单独版权头。Apache License 2.0 文本见 `data_collection/LICENSE-EVA`。采集网页工作流参考 EVA，界面与 Nero 采集控制为本项目实现。

## Lucide

`data_collection/nero_pico_data/static/lucide.min.js` 使用 Lucide 0.468.0，ISC 许可证见 `data_collection/LICENSE-LUCIDE`。

## LeRobot / OpenPI / XRoboToolkit

LeRobot、OpenPI 和 XRoboToolkit 是外部项目。LeRobot 固定 revision 写在 `data_collection/nero_pico_data/lerobot_io.py`，录制和导出共用；OpenPI 不复制入仓库；XRoboToolkit 二进制由 `scripts/fetch_xr.py` 从上游 release 下载并校验 SHA-256。

## 模型与坐标映射参考

- `teleop/models/nero/nero_description.urdf` 来自 [agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf)，revision `f6642ce0d7872c686f29c99e9e10cd23d1d49313`，MIT 许可证在同目录。URDF SHA-256 为 `c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278`。只用于运动学，未打包显示网格。
- [Maniskill_xnero](https://github.com/loopkok/Maniskill_xnero) 提供肩部安装矩阵与身体坐标参考。
- [moonbot_isaacsim](https://github.com/xcs1024/moonbot_isaacsim) 提供 Placo 任务组织与 PICO/XRoboToolkit 坐标处理参考。
- 两个参考项目以及 Isaac Sim、Quest 和其余演示素材不作为运行依赖打包。

第三方许可证仅覆盖各自代码或资产；本整理包没有替项目作者选择新的自有代码开源许可证。

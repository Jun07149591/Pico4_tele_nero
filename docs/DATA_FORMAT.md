# 数据格式

采集阶段保存 Nero 原生 HDF5，包含时间戳、state/action、夹爪宽度、相机 JPEG 和遥测诊断。质检通过且 outcome 为 `success` 的片段才允许导出。

导出使用 OpenPI 选择的 LeRobot v2.1 revision `0cf864870cf29f4738d3ade893e6fd13fbd7cdb5`，启用 `video` 特征：

- `observation.state`：单臂 8 维，七个关节弧度加夹爪宽度米；双臂 16 维，左臂后右臂；
- `action`：已发送的绝对关节目标和夹爪目标；
- `observation.images.<role>`：RGB 视频；
- `task` / `task_index`：网页任务指令。

每个 episode 有一个 `data/chunk-000/episode_XXXXXX.parquet`，每路相机有一个同索引的 H.264 MP4。视频只在 `videos/` 下保存，不生成 `clips/`。导出完成前程序会验证 Parquet 数量、`episode_index`、帧数和 MP4 非空状态，并将源文件 hash 和路径写入 `meta/nero_provenance.json`。

OpenPI 使用时将 `meta/nero_openpi.json` 传给 `scripts/openpi.sh`。脚本要求显式提供外部 OpenPI 路径，不修改 OpenPI 源码。加载器从 spec 文件所在位置解析数据集目录，复制数据集到新电脑后不再依赖旧机器的绝对路径；请保留 `<dataset-home>/<namespace>/<dataset>/meta/nero_openpi.json` 结构。

数据集名称必须符合 `namespace/dataset`，例如 `local/nero_pick_place`。网页会按采集根目录保存名称，刷新后继续使用上次名称；一次导出不会覆盖已有目录。

## OpenPI 微调入口

先按 [OpenPI 官方 README](https://github.com/Physical-Intelligence/openpi#fine-tuning-base-models-on-your-own-data) 建立训练环境、下载模型并准备 GPU。采集环境使用 CPU PyTorch，不包含 OpenPI 的 JAX/GPU 训练依赖。

```bash
export OPENPI_PROJECT_DIR=/path/to/openpi
bash scripts/openpi.sh check --spec /path/to/local/dataset/meta/nero_openpi.json
bash scripts/openpi.sh norm --spec /path/to/local/dataset/meta/nero_openpi.json
bash scripts/openpi.sh train --spec /path/to/local/dataset/meta/nero_openpi.json --exp-name nero_run
```

桥接配置注册为 `pi05_nero_single` 或 `pi05_nero_dual`，基于 OpenPI `pi05_libero`；默认 batch size 8、action horizon 20、关闭 W&B。30 Hz 的 20 帧 action chunk 对应约 0.67 秒。七个关节 action 转成相对当前状态的差值用于训练，夹爪保持绝对宽度；模型输入由 OpenPI 补齐到预训练维度。

Nero 每臂为七关节加一个夹爪，不可直接当作 ALOHA 的六关节配置。夹爪单位是米，例如 `0.03` 为 30 mm。单/双臂训练应使用对应模式的数据集。此次打包验证包含数据转换与加载，不包含模型微调收敛或真机策略执行验证。

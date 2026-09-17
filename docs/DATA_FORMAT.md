# 数据格式

新采集直接写 LeRobot v2.1：每帧 RGB 在后台写入 H.264 MP4，state/action 和索引每约一秒组成一个 Parquet row group 写入磁盘。不会先存 JPEG 再转 PNG，也不会把整段 RGB 图像留在内存里。正常结束时只需排空有界队列、关闭编码器和 Parquet 文件、保存统计量及元数据。

每个采集片段是一个独立的单 episode LeRobot 数据集，内部官方索引为 `0`，目录如下。这样不同频率、成功/失败片段和删除操作可以独立管理；勾选导出时再合并为连续编号的数据集。

```text
<root>/episodes/
  episode_01.h5                  # 时间、state/action、相机时间戳、诊断及缺帧记录
  episode_01.review.json         # 人工质检记录
  episode_01/
    data/chunk-000/episode_000000.parquet
    videos/chunk-000/observation.images.front/episode_000000.mp4
    videos/chunk-000/observation.images.right_wrist/episode_000000.mp4
    meta/info.json
    meta/tasks.jsonl
    meta/episodes.jsonl
    meta/episodes_stats.jsonl
    meta/nero_recording.json     # 结果、诊断来源、缺帧数量
    meta/nero_files.json         # 文件大小与 SHA-256
```

同名 HDF5 是网页的诊断索引，属性 `image_storage=lerobot_video`，不含 JPEG；视频直接用于回放和训练。不能只复制或删除 `.h5`，备份时应保留整个采集根目录。标准 LeRobot 加载器可以直接读取完成的 `episode_01/`；只有质检通过且 outcome 为 `success` 的片段才允许由网页合并导出到训练集。

编码器使用 CPU `libx264`，CRF 18、`veryfast`、GOP 2、关闭 B 帧，通常为 YUV420P，奇数尺寸使用 YUV444P。帧 PTS 固定为 `frame_index / fps`。图像归一化统计采用与 LeRobot 相同的空间降采样，在每帧原始 RGB 上增量累计；统计量形状和计数遵循 v2.1。正常结束前的数据位于 `.inprogress/`，只有视频、表格和元数据全部完成后才发布 HDF5 索引。写入失败保留半成品，不会被列为可导出片段；MP4/Parquet 均需要收尾，断电不能保证当前段可恢复。

录制由用户手动结束，不设固定时长。B/Y 回位时，实际测量状态与已发送回位目标仍作为 state/action 写入当前片段，遥测诊断保留 `home_state`，不会因为回位拆分片段。

新片段的 HDF5 属性 `recording_policy` 为 `manual_finish`。`capture_gaps` 是变长 JSON 字符串数组，每项记录缺失区间的 `start_monotonic`、`end_monotonic`、`skipped_frames`、首次 `reason` 和最近 `last_reason`。短暂数据延迟会重试，真正缺失的帧不生成替代样本，数据恢复后在同一片段继续写入；缺失期间手动结束也保留已经确认的缺失区间。

`timestamp` 是按有效帧序号除以采样频率生成的回放时间；`monotonic` 和 `wall_time_ns` 保存实际采样时间。有缺帧时，实际时间会出现跳跃，回放时长也会短于录制经过时间。非空 `capture_gaps` 会触发质检错误 `capture_data_gaps`，禁止将这种片段当作连续示范导出。旧 HDF5 无此字段时仍可读取和质检。

录制和导出均使用 OpenPI 选择的 LeRobot v2.1 revision `0cf864870cf29f4738d3ade893e6fd13fbd7cdb5` 的官方元数据及特征 API，启用 `video` 特征：

- `observation.state`：单臂 8 维，七个关节弧度加夹爪宽度米；双臂 16 维，左臂后右臂；
- `action`：已发送的绝对关节目标和夹爪目标；
- `observation.images.<role>`：RGB 视频；
- `task` / `task_index`：网页任务指令。

导出时每个 episode 有一个 `data/chunk-000/episode_XXXXXX.parquet`，每路相机有一个同索引的 H.264 MP4。视频只在 `videos/` 下保存，不生成 `clips/`。新数据导出仅校验文件、复制 MP4、重排小型 Parquet 中的 `episode_index`、全局 `index` 和 `task_index`，并用官方 API 重建元数据和统计量，不重新编码视频。复制的视频是独立文件，删除源片段不会影响导出副本。

完整质检会顺序解码视频，检查帧数、时间和尺寸；导出时校验录制文件的 SHA-256、表格与诊断的对应关系以及原有时间对齐检查，避免再次逐像素解码。导出完成前验证文件数及帧数，并将源文件 hash、路径和 `video_reused` 写入 `meta/nero_provenance.json`。报告的 `reused_video_episodes` 表示直接复用的片段数，`converted_legacy_episodes` 表示旧格式转换数。

旧版 HDF5 中的 `images/<role>` JPEG 仍受支持，允许与同频率的新片段一起导出。旧数据导出直接从 JPEG 解码后编码 MP4，不经过临时 PNG，也不修改原始数据。因此旧数据仍有一次转换成本，新采集则在录制过程中完成视频编码。首次质检、大数据集复制和磁盘较慢时仍需要等待，不能保证所有导出都瞬间完成。

OpenPI 使用时将 `meta/nero_openpi.json` 传给 `scripts/openpi.sh`。脚本要求显式提供外部 OpenPI 路径，不修改 OpenPI 源码。加载器从 spec 文件所在位置解析数据集目录，复制数据集到新电脑后不再依赖旧机器的绝对路径；请保留 `<dataset-home>/<namespace>/<dataset>/meta/nero_openpi.json` 结构。

数据集名称必须符合 `namespace/dataset`，例如 `local/nero_pick_place`。网页会按采集根目录保存名称，刷新后继续使用上次名称；一次导出不会覆盖已有目录。

原始片段支持在网页勾选删除，同时删除同名 LeRobot 目录、HDF5 诊断和 `.review.json`。删除前会把已使用的最大编号保存到录制根目录的 `episode_sequence.json`，后续新片段不复用已删除编号；复制完整原始数据集时保留这个文件。旧录制目录首次使用此功能时会从已有片段及 `.inprogress` 自动建立编号记录。删除原始片段不修改已完成的导出目录及其来源记录。

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

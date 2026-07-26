# 数据采集与数据集生成

这份文档只覆盖：

- 采集卡/UVC 相机取视频
- 原始视频会话录制
- 直接上传视频到 Ultralytics Platform
- 可选的离线抽帧导出

它**不包含**：

- 仓库内训练
- 仓库内标注 UI
- 控制链联调

## 1. 确认采集卡设备

先确认采集卡对应的 `/dev/videoN`：

```bash
v4l2-ctl --list-devices
```

再查看采集卡当前能力：

```bash
v4l2-ctl --device=/dev/video0 --all
```

如果画面链路还没确认，先用预览入口验证：

```bash
./build/tool_preview config/capture_red_20m.yaml
```

## 2. 录制原始视频会话

推荐使用专门的采集配置：

```text
config/capture_red_20m.yaml
```

这个配置当前默认面向 UGREEN 采集卡直读节点：

- `v4l2.device_path=/dev/v4l/by-id/...`
- 如果你机器上的 UGREEN 稳定路径不同，先改这里再录制
- 录制参数位于 `record.output_root` / `record.duration_seconds` / `record.*_tag`

它默认：

- `1920x1080`
- `60 FPS`
- `mjpeg`
- 打开窗口显示和 overlay 绘制

录制命令：

```bash
./build/tool_record config/capture_red_20m.yaml
```

也可以继续用 CLI 参数覆盖 YAML 里的录制设置：

```bash
./build/tool_record \
  config/capture_red_20m.yaml \
  /abs/path/to/sessions \
  30 \
  indoor_lab \
  plain_wall \
  20m \
  red
```

如果不传第 2 个参数，默认会话根目录是仓库根目录下的 `videos/`。
`tool_record` 会在运行时强制使用 `yuyv` 取图；如果你要排查配置本身的取图问题，优先用 `tool_preview` 单独验证。
`tool_record` 默认打开原始预览窗口；如需关闭预览（避免高分辨率录制时额外的 BGR 原始管道影响帧率），设置 `LASER_RECORD_PREVIEW=0`。
运行后终端会持续占用，等时长结束或按 `Ctrl+C` 才会优雅收尾并写完 `raw.mp4` / `session.yaml` / `notes.txt`。

参数含义：

1. 配置文件路径
2. 会话根目录
3. 录制时长，单位秒
4. `lighting_tag`
5. `background_tag`
6. `distance_tag`
7. `target_color`

每次录制会生成：

```text
<session_root>/<session_id>/
├── raw.mp4
├── session.yaml
└── notes.txt
```

其中：

- `raw.mp4`
  - 原始视频，默认写为 `H.264/avc1`，优先直接上传 Ultralytics Platform；也可作为本地离线抽帧输入
- `session.yaml`
  - 记录设备、分辨率、帧率、场景标签
- `notes.txt`
  - 供人工补充污染光、遮挡、异常情况

## 3. 采集组织规则

不要把所有视频混在一起。应按完整会话组织：

- `train`
  - 常见场景，多数视频会话
- `val`
  - 独立少量会话，用于选模型
- `test`
  - 完全独立场次，最好换日期、换背景或换光照

同一条原始视频不能同时出现在多个 split。

建议每个 split 都覆盖：

- 目标完整清晰
- 目标偏航/俯仰
- 部分遮挡
- 背景污染光
- 无目标负样本

## 4. 直接上传视频到 Ultralytics Platform

当前推荐工作流是：

1. 用 `tool_record` 录出完整会话
2. 保留 `session.yaml` 和 `notes.txt` 作为场景记录
3. 直接把 `raw.mp4` 上传到 Ultralytics Platform
4. 在平台内完成抽帧、标注和训练

如果你手里已有旧的 `mp4v` 会话，可以先原地转码：

```bash
./build/tool_transcode /abs/path/to/videos/<session_id>
```

这样做的优点是：

- 保留完整原始视频，后续可重复抽帧
- 不需要在仓库里先落一份本地待标注图片
- split 可以继续按“完整会话”管理，而不是按零散图片管理

## 5. 离线抽帧（可选备用）

把单个会话导出为待标注图片：

```bash
./build/tool_export \
  /abs/path/to/sessions/<session_id> \
  /abs/path/to/dataset_root \
  train \
  200
```

参数含义：

1. 单个会话目录
2. 数据集根目录
3. split，必须是 `train` / `val` / `test`
4. 抽帧间隔，单位毫秒

默认推荐：

- `200 ms` 一帧
- 也就是约 `5 FPS`

导出结果：

```text
<dataset_root>/
├── images/
│   ├── train/
│   ├── val/
│   └── test/
└── manifests/
    └── <session_id>_<split>_export_manifest.csv
```

manifest 中会保留：

- `source_session_id`
- `source_timestamp_ms`
- `split`
- `blur_score`
- 导出图片尺寸

## 6. 标注与训练

仓库不负责本地训练。

如果你走推荐链路：

1. 上传 `raw.mp4`
2. 在 Ultralytics Platform 内抽帧、标注、训练

如果你走本地备用链路：

1. 删除完全不可用帧
2. 用外部工具做 bbox 标注
3. 导出 Ultralytics 检测格式：

```text
images/train|val|test
labels/train|val|test
data.yaml
```

4. 上传到 Ultralytics Platform 训练

第一阶段推荐单类：

```text
target_red
```

## 7. 采集注意事项

- 优先固定曝光、固定增益、固定白平衡
- 如果采集卡链路允许，优先高快门而不是高增益
- 先保证原始画面干净，再考虑是否需要本地抽帧
- 不要保存带 overlay 的训练图
- 每次换场地、光照、背景、距离都应单独录成新会话
- `notes.txt` 里至少记录污染光来源、目标运动方式、异常片段

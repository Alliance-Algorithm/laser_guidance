# 相机参数调试记录（2026-08-03）

## 问题

第三阶段（stage-3，无色反制模块）模型检不出 colorless（class 3）目标：
画面中目标在，但模型输出 0 候选（`detected=false`）。

## 排查过程与排除项

| 变量 | 结论 |
|---|---|
| 模型文件（`models/exp-900-901.engine`，7/29 生成） | 未变，对 7/23 历史帧能检出 colorless（score 0.85+，21 帧实证） |
| 推理后端 | TensorRT 与 ONNX 结果一致，均 0 候选/均可检出历史帧 |
| 感知代码（`src/vision/`、`src/capture/`） | 8/2 检出成功（d89c8bf/8ddbc7b）至今零改动；回退分支实测同样 0 候选 |
| 裁判系统逻辑（RefereeLink/HitProgress） | 完全不参与模型推理，不影响识别 |
| 白平衡 | AWB 与固定 WB（R1450/G900/B2300）均 0 候选 |
| 曝光/增益 | 20000us/16.9 与 500us/10 组合均 0 候选 |
| 取图/推理链路 | shm 帧非黑（mean 90+）、detect_dump 同链路 0 候选、同帧直接喂模型 0 候选 |
| **环境光照** | **暗环境（帧 mean <~40）→ colorless 检出成功（score 0.81+）；亮环境（mean 90+）→ 0 候选** |

## 根因

**环境光照**。无色目标是暗场中的深色目标（7/23 训练采集为暗环境，目标周围背景亮度 6~25）。
亮环境下目标与背景对比度/外观结构偏离训练分布，模型输出 0 候选。
第三阶段参数（20000us / gain 16.9 / AWB）+ 暗环境实测 colorless 检出成功（score 0.813）。

## 相机参数残留（排查中发现的重要陷阱）

Hik 相机会**跨会话保留参数状态**：

- 相机空闲时（无 daemon）用 `tool_cam_params_dump` 读到 MVS 客户端残留参数：
  `exposure 2000us / gain 16 / 固定白平衡 1231-1024-3193 / gamma 0`
- `HikBackend::open()` 只重置 exposure/gain/framerate/白平衡；
  **gamma、对比度、饱和度等图像参数从不重置**，MVS 改过会残留并影响画面特征
- daemon 运行时相机被独占，dump 工具会报 "Access denied to device"

排查相机问题时先确认：**daemon 未运行时** dump 的参数是残留状态，不等于 daemon 运行时参数。

## 调试工具

| 工具 | 用途 |
|---|---|
| `build/tool_cam_params_dump [device_id]` | 读取相机当前参数（需相机空闲） |
| `build/tool_cam_grab [device_id] [--exposure] [--gain] [--gamma] [--wb-r/--wb-g/--wb-b] [--out]` | 设置参数并抓一帧到磁盘，配合 `tool_detect_dump` 隔离参数 vs 模型问题 |

运行前需 Hik MVS 运行时环境：
`export LD_LIBRARY_PATH=vendor/hikcamera/src/sdk/lib:$LD_LIBRARY_PATH`

## 复现/验证

```bash
# 第三阶段参数 + 暗环境 → colorless 检出
./build/tool_cam_grab DB0864607 --exposure 20000 --gain 16.9 --out /tmp/f.png
./build/tool_detect_dump models/exp-900-901.engine /tmp/f.png
# 期望: candidates=1, class=colorless, score>0.8（环境需暗，帧 mean<~40）
```

## 遗留事项

- 亮环境无色目标检不出：需补亮场训练数据重训，或光学手段（遮光/减光镜）控制目标区域环境光
- `competition-laser` 的 ffplay 启动时序问题（RTP 流未就绪即拉流，ffplay 报
  `not enough frames to estimate rate` 后退场并连带杀 daemon）：未修复，临时用 `LASER_HEADLESS=1`

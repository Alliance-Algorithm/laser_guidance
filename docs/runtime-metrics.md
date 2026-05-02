# Runtime Metrics Schema

## Format: JSONL (one JSON object per line, per frame)

## Fields

### Timestamps (nanoseconds, monotonic clock)
| Field | Description |
|-------|-------------|
| `frame_id` | Monotonically increasing frame counter |
| `t_capture_ns` | Camera capture timestamp |
| `t_worker_start_ns` | Worker thread received frame |
| `t_preprocess_done_ns` | Preprocessing complete |
| `t_infer_start_ns` | TensorRT enqueue called |
| `t_infer_done_ns` | TensorRT output ready on host |
| `t_postprocess_done_ns` | Adapter/postprocess complete |
| `t_publish_ns` | Observation published |

### Ages / Durations (ms)
| Field | Description |
|-------|-------------|
| `observation_age_ms` | `t_publish - t_capture` |
| `input_age_at_worker_start_ms` | `t_worker_start - t_capture` |
| `input_age_at_infer_start_ms` | `t_infer_start - t_capture` |
| `preprocess_ms` | Preprocessing duration |
| `inference_ms` | TensorRT H2D+enqueue+D2H |
| `postprocess_ms` | Adapter/postprocess duration |

### Drops / Overwrites (counters, cumulative)
| Field | Description |
|-------|-------------|
| `queue_overwrite_count` | Times a new frame overwrote old before consumption |
| `stale_drop_count` | Frames dropped because age > max_input_age_ms |
| `debug_drop_count` | Debug frames skipped (renderer too slow) |
| `record_drop_count` | Recording frames skipped (disk too slow) |

### Detection
| Field | Description |
|-------|-------------|
| `detected` | `true/false` |
| `class_id` | `0=Red, 1=Blue, 2=Purple` |
| `confidence` | Detection confidence |
| `center_x`, `center_y` | Bbox center (pixels) |
| `bbox_x1`, `bbox_y1`, `bbox_x2`, `bbox_y2` | Bbox corners (pixels) |

### HIT State
| Field | Description |
|-------|-------------|
| `hit_state` | `none`, `candidate`, `confirmed` |
| `hit_consecutive` | Consecutive Purple frame count |

### Metadata
| Field | Description |
|-------|-------------|
| `backend` | `tensorrt`, `ort_cuda`, `mock`, etc. |
| `engine_path` | Path to loaded `.engine` file |
| `unity_mode` | `off`, `standalone_60fps`, `standalone_30fps` |
| `gpu_util_pct` | GPU utilization percentage |
| `gpu_mem_used_mb` | GPU memory used |

## Default Thresholds

| Parameter | Value |
|-----------|-------|
| `max_input_age_ms` | 25 |
| `max_observation_age_ms` | 35 |
| `warmup_frames` | 30 |
| `max_infer_fps` | 60 |

## Acceptance Metrics

- **p50/p90/p95/p99/max** observation age — primary latency metric.
- **NOT** average FPS.
- Overload success: drop/overwrite counters rise while observation age stays bounded.
- Stale reasons must be distinguishable (too old before worker, too old before inference, too old after inference).

## Example Log Line

```json
{"frame_id":123,"t_capture_ns":1234567890000,"t_worker_start_ns":1234567895000,"t_preprocess_done_ns":1234567900000,"t_infer_start_ns":1234567901000,"t_infer_done_ns":1234567903500,"t_postprocess_done_ns":1234567904000,"t_publish_ns":1234567904500,"observation_age_ms":14.5,"input_age_at_worker_start_ms":5.0,"input_age_at_infer_start_ms":11.0,"preprocess_ms":5.0,"inference_ms":2.5,"postprocess_ms":0.5,"queue_overwrite_count":3,"stale_drop_count":0,"debug_drop_count":1,"record_drop_count":0,"detected":true,"class_id":1,"confidence":0.92,"center_x":320,"center_y":240,"bbox_x1":300,"bbox_y1":220,"bbox_x2":340,"bbox_y2":260,"hit_state":"none","hit_consecutive":0,"backend":"tensorrt","engine_path":"models/target_fp16_1x3x640x640.engine","unity_mode":"standalone_60fps","gpu_util_pct":45,"gpu_mem_used_mb":1200}
```

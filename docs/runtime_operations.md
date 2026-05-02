# Freshness Runtime Operations

## Modes

### Guidance Mode (competition / real-time)

```yaml
runtime:
  mode: guidance
  max_input_age_ms: 25
  max_observation_age_ms: 35
  max_infer_fps: 60
  warmup_frames: 30
  engine_path: models/target_fp16_1x3x640x640.engine
  hit_confirm_frames: 3
  hit_release_frames: 5
  debug_enabled: false
  debug_max_fps: 30
  record_enabled: false
  record_queue_size: 16
```

**Rules**:
- Debug and recording must be lossy. If they fall behind, they drop frames.
- Debug/recording must never block the Capture→Worker hard path.
- TensorRT engine is loaded at startup, never built at runtime.

### Collection Mode (data gathering / training)

```yaml
runtime:
  mode: collection
  record_enabled: true
  record_queue_size: 32
  max_input_age_ms: 100
```

**Rules**:
- Recording can be larger and more permissive.
- Latency is relaxed; freshness is less critical.
- Original `videos/<session_id>/` sessions remain the source of truth.

## Build Commands

### Default Build (no TensorRT)

```bash
cmake -S . -B build/default -DCMAKE_BUILD_TYPE=Release
cmake --build build/default --parallel
ctest --test-dir build/default --output-on-failure
```

### TensorRT Build

```bash
cmake -S . -B build/tensorrt -DCMAKE_BUILD_TYPE=Release \
  -DRMCS_LASER_GUIDANCE_WITH_TENSORRT=ON
cmake --build build/tensorrt --parallel
```

### TensorRT Engine Build (offline)

```bash
trtexec \
  --onnx=models/target.onnx \
  --saveEngine=models/target_fp16_1x3x640x640.engine \
  --fp16 \
  --optShapes=images:1x3x640x640 \
  --skipInference
```

## MP4 Upload Packaging

Copy recorded sessions for labeling upload:

```bash
python - <<'PY'
from pathlib import Path, csv, hashlib
root = Path('.')
upload = root / 'videos' / 'label_upload' / 'raw_sessions'
upload.mkdir(parents=True, exist_ok=True)
rows = []
for mp4 in sorted(root.glob('videos/*/raw.mp4')):
    sid = mp4.parent.name
    dst = upload / f'{sid}_raw.mp4'
    dst.write_bytes(mp4.read_bytes())
    h = hashlib.sha256(dst.read_bytes()).hexdigest()
    rows.append({'session_id': sid, 'upload_mp4': str(dst), 'file_size': dst.stat().st_size, 'sha256': h})
with (upload.parent / 'upload_manifest.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=['session_id','upload_mp4','file_size','sha256'])
    w.writeheader(); w.writerows(rows)
print(f'{len(rows)} sessions packaged')
PY
```

## Interpreting Latency Logs

Key metrics (JSONL, one line per frame):

| Field | Meaning |
|-------|---------|
| `observation_age_ms` | Frame age at publish time. Primary metric. |
| `inference_ms` | TensorRT H2D+enqueue+D2H time |
| `queue_overwrite_count` | Times a new frame replaced old before processing |
| `stale_drop_count` | Frames dropped for exceeding max age |
| `hit_state` | `none`, `candidate`, or `confirmed` |

**Acceptance**:
- p95/p99 observation_age_ms must stay bounded.
- Overwrite/drop counters rising is OK (system degrades by reducing rate, not adding latency).
- Average FPS is NOT the primary metric.

## Known Limitations

- Unity in Editor mode may distort CPU/GPU behavior; use standalone player for benchmarks.
- `trtexec` heavy benchmarks must not run while Unity is under high render load.
- The first TensorRT inference after engine load may have warmup jitter; warmup frames are configured but not validated until a real model is available.

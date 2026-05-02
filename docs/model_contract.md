# Model Contract: Target Detection

> Version: 1.0-draft
> Status: awaiting-onnx-export
> Last updated: 2026-05-02

## Purpose

This document defines the frozen model contract for the first deployable target-detection model used by `rmcs_laser_guidance`. It is the source of truth for ONNX export, TensorRT engine generation, and C++ runtime integration.

## Input

| Field     | Value                    | Notes                                 |
|-----------|--------------------------|---------------------------------------|
| Name      | `images`                 | Must confirm after ONNX export        |
| Shape     | `1x3x640x640`            | Fixed shape, Batch=1                  |
| Layout    | `NCHW`                   | Channels first                        |
| Dtype     | `FP32` input             | TensorRT runtime may use FP16 kernels |
| Color     | `[DECISION NEEDED: RGB or BGR — confirm after export]` | |
| Normalization | `[DECISION NEEDED: 0~1 range or mean/std — confirm after export]` | |
| Dynamic shape | Not supported in v1 | Fixed shape only                 |

## Output

| Field         | Value                                | Notes |
|---------------|--------------------------------------|-------|
| Class count   | 3 | `0=Red`, `1=Blue`, `2=Purple` |
| Class names   | `Red`, `Blue`, `Purple` | |
| NMS location  | `[DECISION NEEDED: inside model or C++ postprocess — confirm after export]` | |
| Bbox format   | `[DECISION NEEDED: xyxy / cxcywh / other — confirm after export]` | |
| Output tensors | `[DECISION NEEDED: exact YOLO output tensor contract after export]` | Tensor names, shapes, and semantics must be confirmed after `.onnx` export |
| Confidence    | Float                                | |

> **Important**: Single model with 3 classes shares a common backbone. Purple is a model class (id=2), not a separate model. Runtime filters by `match_color` config: red team accepts Blue+Purple, rejects Red; blue team accepts Red+Purple, rejects Blue.

## Runtime Color Filter

```text
config: match_color = red | blue

match_color=red:
  class=Blue  → accept (target)
  class=Red   → reject (own team)
  class=Purple → accept (HIT, both teams)

match_color=blue:
  class=Red   → accept (target)
  class=Blue  → reject (own team)
  class=Purple → accept (HIT, both teams)
```

## Runtime Artifact

| Artifact               | Role                             | Format      |
|------------------------|----------------------------------|-------------|
| ONNX model             | Export / interchange / validation | `.onnx`     |
| TensorRT engine        | Prebuilt runtime artifact         | `.engine`   |

**Rules**:

- The `.engine` file must be built **offline**, before the application starts, using `trtexec` or equivalent tooling.
- The application runtime must **never** build engines from ONNX in v1.
- On engine load failure or shape mismatch, the runtime must print explicit input/output metadata and fail clearly, never silently return empty detections.
- Engine filename convention: `target_fp16_1x3x640x640.engine`

## Build Command Template

```bash
trtexec \
  --onnx=models/target.onnx \
  --saveEngine=models/target_fp16_1x3x640x640.engine \
  --fp16 \
  --optShapes=images:1x3x640x640 \
  --skipInference
```

> Replace `images` if exported ONNX input name differs.

## Validation

Before a model is accepted for runtime deployment:

1. Export `.onnx` from training framework.
2. Validate ONNX loads correctly (shape, input/output names).
3. Compare ONNX reference output against a curated validation set (`test_data/model_validation/`).
4. Build FP16 `.engine` with the template command above.
5. Compare TensorRT engine output against ONNX reference output on the same validation set.
6. Run a short benchmark (`trtexec --loadEngine ...`).
7. Run latency validation under Unity-off, Unity 60 FPS, and Unity 30 FPS conditions.

## Hit Detection (model class + temporal hysteresis)

Purple is a model output class (id=2). HIT state uses temporal hysteresis on consecutive Purple detections:

- Purple class triggers candidate HIT state.
- **Hysteresis defaults**:
  - `purple_ratio_on = 0.18`
  - `purple_ratio_off = 0.10`
  - `confirm_frames = 3`
  - `release_frames = 5`
- HIT confirms after N consecutive frames above the on-threshold.
- HIT releases after M consecutive frames below the off-threshold.
- Flicker between thresholds does not toggle HIT rapidly.

## Versioning

Each model artifact should carry:

| Field           | Example                        |
|-----------------|--------------------------------|
| Model name      | `yolo26n_target_v1`            |
| ONNX filename   | `target_v1.onnx`               |
| Engine filename | `target_v1_fp16_1x3x640x640.engine` |
| Input shape     | `1x3x640x640`                  |
| Precision       | `FP16`                         |
| SHA-256         | `[to be filled after export]`  |
| Export date     | `[to be filled after export]`  |
| Training dataset version | `[to be filled]`       |

## Out of Scope (for this model contract and v1 runtime)

- Tracker, solver, planner.
- `/tf`, `/gimbal/*`, `fire_control`.
- Dynamic batch or dynamic shape.
- Multi-context or multi-stream TensorRT runtime.
- INT8, CUDA Graph runtime, CPU affinity tuning.
- Automatic GPU/CPU backend switching.
- Local model training.
- Separate ROI color classifier for Red/Blue/Purple target identification in v1 (the model already predicts `Red`, `Blue`, `Purple`; runtime only applies `match_color` filtering and Purple HIT hysteresis).

## References

- `config/capture_red_20m.yaml` — recommended recording configuration.
- `README.md` — project build and example documentation.
- `docs/architecture.md` — current architecture boundaries.
- `AGENTS.md` — repository constraints and phase scope.

# Model Contract: Target Detection

> Version: 1.1
> Status: active (`models/exp-2.onnx` / `models/exp-2.engine`)
> Last updated: 2026-07-29

## Purpose

This document defines the model contract for the deployable target-detection model used by `rmcs_laser_guidance`. It is the source of truth for ONNX export, TensorRT engine generation, and C++ runtime integration.

## Input

| Field     | Value                    | Notes                                 |
|-----------|--------------------------|---------------------------------------|
| Name      | `images`                 | Ultralytics YOLO export               |
| Shape     | `1x3x1280x1280`          | Current `exp-2` imgsz; Batch=1        |
| Layout    | `NCHW`                   | Channels first                        |
| Dtype     | `FP32` input             | TensorRT runtime may use FP16 kernels |
| Color     | RGB                      | Ultralytics default                   |
| Normalization | `0~1`                | Divide by 255                         |
| Dynamic shape | Export may be dynamic; runtime uses fixed batch-1 letterbox | |

## Output

| Field         | Value                                | Notes |
|---------------|--------------------------------------|-------|
| Class count   | 4 | From ONNX metadata `names` |
| Class ids     | `0=Red(红色)`, `1=Blue(蓝色)`, `2=Purple(紫色)`, `3=Colorless(无色)` | Canonical runtime mapping |
| Class names   | `Red`, `Blue`, `Purple`, `Colorless` | English aliases used in code/docs |
| NMS location  | C++ postprocess (`ModelAdapter`) when export has `nms=False` | |
| Bbox format   | Model input space, then unletterboxed to original frame pixels | |
| Confidence    | Float                                | |

> **Important**: Single model with 4 classes. Purple is class id **2** (HIT), and Colorless is class id **3**. The runtime preserves all four classes for continuous guidance. In competition mode, Purple and Colorless are additionally consumed by the countermeasure state machine; they are not rejected from guidance.

## Runtime Guidance Classes

```text
class=Red(0)       → retain for guidance
class=Blue(1)      → retain for guidance
class=Purple(2)    → retain for guidance and countermeasure progress
class=Colorless(3) → retain for guidance and countermeasure confirmation/progress
```

`enemy_color` remains available as a runtime control value, but it does not
remove model detections from the physical guidance path. Target selection and
countermeasure state consume the same retained detection batch.

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

Purple is a model output class (**id=2**). HIT state uses temporal hysteresis on consecutive Purple detections:

- Purple (`class_id == 2`) triggers candidate HIT state.
- Colorless (`class_id == 3`) does not add Purple progress. During the first
  three countermeasure stages it confirms a countermeasure only after the
  Purple `HitProgress` threshold is reached; during difficulty-3 stages it is
  the continuing illumination class used for progress.
- **Hysteresis defaults** (`HitStateMachine` / runtime config):
  - `hit_confirm_frames = 3`
  - `hit_release_frames = 5`
  - score gate typically `>= 0.25`
- HIT confirms after N consecutive Purple frames.
- HIT releases after M consecutive non-Purple frames.
- Flicker does not toggle HIT rapidly.

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
- Separate ROI color classifier (the model already predicts `Red`, `Blue`, `Purple`, `Colorless`; runtime passes all model classes to guidance and uses Purple/Colorless for countermeasure state).

## References

- `config/capture_red_20m.yaml` — recommended recording configuration.
- `README.md` — project build and example documentation.
- `docs/architecture.md` — current architecture boundaries.
- `docs/AGENTS.md` — repository constraints and phase scope.

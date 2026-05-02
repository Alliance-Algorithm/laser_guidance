# Model Validation Fixtures

This document defines the validation fixture layout and manifest schema used to compare:

- ONNX reference output
- TensorRT engine output
- Runtime postprocess behavior
- Purple/HIT temporal behavior after detection

It is designed to work alongside the current dataset export flow and the already extracted frame directory under `dataset/train/images/`.

## Fixture Root

```text
test_data/model_validation/
```

Recommended layout:

```text
test_data/model_validation/
├── images/
│   ├── normal_target/
│   ├── purple_hit_target/
│   ├── negative/
│   ├── reflection_led/
│   ├── motion_blur/
│   ├── overexposed/
│   ├── far_20m/
│   ├── partial_occlusion/
│   └── purple_non_target/
└── manifests/
    └── validation_manifest.csv
```

The images can be sampled from:

- `dataset/train/images/` for broad coverage
- future curated validation-only exports from `videos/<session_id>/raw.mp4`

## Manifest Schema

The validation manifest must be a CSV with this exact header:

```csv
image_path,category,expected_detected,expected_color_state,notes,source_session_id,source_timestamp_ms
```

### Field Meanings

| Field | Meaning |
|---|---|
| `image_path` | Relative path to the validation image |
| `category` | One of the required hard-case categories below |
| `expected_detected` | `true` or `false` |
| `expected_color_state` | `Red`, `Blue`, `Purple`, `Unknown`, or `None` |
| `notes` | Free-form context for edge cases |
| `source_session_id` | Source session directory name, e.g. `20260502T093007` |
| `source_timestamp_ms` | Timestamp of the frame in the original recording |

## Required Hard-Case Categories

Every validation set should include these categories:

- `normal_target`
- `purple_hit_target`
- `negative`
- `reflection_led`
- `motion_blur`
- `overexposed`
- `far_20m`
- `partial_occlusion`
- `purple_non_target`

### Why These Categories Matter

- `normal_target`: standard red/blue target detections
- `purple_hit_target`: verifies class `Purple` and HIT hysteresis downstream
- `negative`: confirms no-target frames stay empty
- `reflection_led`: guards against bright false positives
- `motion_blur`: tests robustness under fast movement
- `overexposed`: tests color/class stability in poor lighting
- `far_20m`: core competition-distance behavior
- `partial_occlusion`: verifies partial visibility handling
- `purple_non_target`: ensures purple background objects do not become fake targets

## Placeholder / No-Label Workflow While Training Is In Progress

While the model is still training, the manifest can contain placeholder expectation rows.

Example:

```csv
image_path,category,expected_detected,expected_color_state,notes,source_session_id,source_timestamp_ms
images/normal_target/20260502T093007_0001.jpg,normal_target,,,"pending review",20260502T093007,1000
```

Rules:

- empty `expected_detected` or `expected_color_state` means "not finalized yet"
- placeholder rows are allowed before ONNX export is ready
- once the model contract is frozen, placeholder rows should be resolved for the core validation subset

## Initial Validation Strategy

Before the first TensorRT runtime deployment, build a compact, high-signal validation subset:

- 50–100 `normal_target`
- 30–50 `purple_hit_target`
- 30–50 `negative`
- 20+ examples from each failure-oriented category

This subset is for correctness checks.

The full extracted `dataset/train/images/` directory remains training/labeling material, not the primary correctness benchmark set.

## Relationship to Runtime Validation

This manifest is used to check:

1. whether ONNX and TensorRT both detect the same frames
2. whether `Red/Blue/Purple` class decisions are stable
3. whether no-target frames remain empty
4. whether runtime postprocess and Purple/HIT logic behave as expected

## Current Notes

- Training contract is currently `Red(0)`, `Blue(1)`, `Purple(2)`.
- Purple is a model class, not a separate ROI color classifier in v1.
- `match_color` filtering still applies at runtime:
  - `match_color=red` → accept `Blue` + `Purple`
  - `match_color=blue` → accept `Red` + `Purple`

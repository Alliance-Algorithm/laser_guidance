# TensorRT Build and Benchmark Commands

## Environment Check

```bash
command -v trtexec
/opt/cuda/bin/nvcc --version
ldconfig -p | grep libnvinfer
```

Expected: `trtexec` found, CUDA 13.2, `libnvinfer.so` listed.

---

## Build Engine (offline, before application runtime)

```bash
trtexec \
  --onnx=models/target.onnx \
  --saveEngine=models/target_fp16_1x3x640x640.engine \
  --fp16 \
  --optShapes=images:1x3x640x640 \
  --skipInference
```

**Notes**:
- Replace `images` if the exported ONNX input name differs.
- `--skipInference` builds the engine only; no inference is run.
- This command can be heavy on GPU. Do not run while Unity is uncapped.

---

## Benchmark (with prebuilt engine)

```bash
trtexec \
  --loadEngine=models/target_fp16_1x3x640x640.engine \
  --shapes=images:1x3x640x640 \
  --warmUp=500 \
  --duration=20 \
  --percentile=50,90,95,99
```

**Notes**:
- Run after the engine is built.
- Run in a maintenance window (Unity closed or idle).
- Output includes p50/p90/p95/p99 latency percentiles.

---

## Rules

1. The application runtime must **never** build engines from ONNX in v1.
2. Heavy benchmarks must not run while Unity is uncapped or under high render load.
3. If exported ONNX input name is not `images`, update `--optShapes` and `--shapes` accordingly.

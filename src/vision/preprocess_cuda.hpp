// src/vision/preprocess_cuda.hpp
// GPU-accelerated preprocessing: BGR8 HWC → float32 RGB CHW with bilinear
// resize + letterbox pad (114/255) — all in one kernel pass.
// Called from TensorRTEngine::run_from_bgr() to replace CPU preprocess_blob.
#pragma once

#include <cstddef>
#include <cstdint>

#include <cuda_runtime_api.h>

namespace rmcs_laser_guidance {

// Launch the fused preprocess kernel.
// src_bgr : device pointer, uint8 BGR packed HWC [src_h, src_w, 3]
// dst_chw : device pointer, float32 RGB CHW [3, out_h, out_w] — caller owns allocation
// Returns 0 on success, non-zero CUDA error code on failure.
int preprocess_cuda_bgr_to_chw(
    const unsigned char* src_bgr,
    int src_w, int src_h,
    float* dst_chw,
    int out_w, int out_h,
    cudaStream_t stream);

// Returns the padded geometry for given source/output dimensions.
// scaled_w, scaled_h : content area after letterbox scale
// pad_left, pad_top  : pixel offset of content inside the out_w × out_h canvas
void preprocess_cuda_letterbox_params(
    int src_w, int src_h,
    int out_w, int out_h,
    int& scaled_w, int& scaled_h,
    int& pad_left, int& pad_top);

} // namespace rmcs_laser_guidance

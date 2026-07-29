// src/vision/preprocess_cuda.cu
// Fused CUDA kernel: BGR8 HWC → float32 RGB CHW
// Operations (one kernel pass):
//   1. bilinear resize (source → scaled content)
//   2. letterbox pad at 114/255
//   3. uint8→float /255  normalise
//   4. HWC→CHW repack + BGR→RGB channel swap

#include "vision/preprocess_cuda.hpp"

#include <algorithm>
#include <cmath>

namespace rmcs_laser_guidance {

// ---------------------------------------------------------------------------
// kernel
// ---------------------------------------------------------------------------
__global__ void k_bgr_letterbox_to_rgb_chw(
    const unsigned char* __restrict__ src,   // BGR8 HWC [src_h, src_w, 3]
    int src_w, int src_h,
    float* __restrict__ dst,                 // float32 RGB CHW [3, out_h, out_w]
    int out_w, int out_h,
    int scaled_w, int scaled_h,
    int pad_left, int pad_top,
    float inv_sx,   // (src_w-1)/(scaled_w-1) — map scaled-x → source-x
    float inv_sy)   // (src_h-1)/(scaled_h-1)
{
    const int ox = static_cast<int>(blockIdx.x) * 32 + static_cast<int>(threadIdx.x);
    const int oy = static_cast<int>(blockIdx.y) * 32 + static_cast<int>(threadIdx.y);
    if (ox >= out_w || oy >= out_h) return;

    const int plane = out_h * out_w;
    const int dst_r = 0 * plane + oy * out_w + ox;
    const int dst_g = 1 * plane + oy * out_w + ox;
    const int dst_b = 2 * plane + oy * out_w + ox;

    const int ix = ox - pad_left;
    const int iy = oy - pad_top;

    if (ix < 0 || iy < 0 || ix >= scaled_w || iy >= scaled_h) {
        constexpr float kPad = 114.0f / 255.0f;
        dst[dst_r] = kPad;
        dst[dst_g] = kPad;
        dst[dst_b] = kPad;
        return;
    }

    // bilinear sample in source space
    const float sx = static_cast<float>(ix) * inv_sx;
    const float sy = static_cast<float>(iy) * inv_sy;

    const int x0 = max(0, static_cast<int>(floorf(sx)));
    const int y0 = max(0, static_cast<int>(floorf(sy)));
    const int x1 = min(src_w - 1, x0 + 1);
    const int y1 = min(src_h - 1, y0 + 1);

    const float fx = sx - floorf(sx);
    const float fy = sy - floorf(sy);
    const float w00 = (1.0f - fx) * (1.0f - fy);
    const float w01 =         fx  * (1.0f - fy);
    const float w10 = (1.0f - fx) *         fy;
    const float w11 =         fx  *         fy;

    const unsigned char* p00 = src + (y0 * src_w + x0) * 3;
    const unsigned char* p01 = src + (y0 * src_w + x1) * 3;
    const unsigned char* p10 = src + (y1 * src_w + x0) * 3;
    const unsigned char* p11 = src + (y1 * src_w + x1) * 3;

    // memory layout: B=ch0, G=ch1, R=ch2
    const float fB = (p00[0]*w00 + p01[0]*w01 + p10[0]*w10 + p11[0]*w11) * (1.0f/255.0f);
    const float fG = (p00[1]*w00 + p01[1]*w01 + p10[1]*w10 + p11[1]*w11) * (1.0f/255.0f);
    const float fR = (p00[2]*w00 + p01[2]*w01 + p10[2]*w10 + p11[2]*w11) * (1.0f/255.0f);

    // write as RGB CHW (model expects RGB)
    dst[dst_r] = fR;
    dst[dst_g] = fG;
    dst[dst_b] = fB;
}

// ---------------------------------------------------------------------------
// host-side helpers
// ---------------------------------------------------------------------------
void preprocess_cuda_letterbox_params(
    int src_w, int src_h,
    int out_w, int out_h,
    int& scaled_w, int& scaled_h,
    int& pad_left, int& pad_top)
{
    const float sx = static_cast<float>(out_w) / static_cast<float>(src_w);
    const float sy = static_cast<float>(out_h) / static_cast<float>(src_h);
    const float scale = (sx < sy) ? sx : sy;
    scaled_w  = static_cast<int>(std::lround(src_w * scale));
    scaled_h  = static_cast<int>(std::lround(src_h * scale));
    scaled_w  = scaled_w  < 1 ? 1 : scaled_w;
    scaled_h  = scaled_h  < 1 ? 1 : scaled_h;
    pad_left  = (out_w - scaled_w) / 2;
    pad_top   = (out_h - scaled_h) / 2;
}

int preprocess_cuda_bgr_to_chw(
    const unsigned char* src_bgr,
    int src_w, int src_h,
    float* dst_chw,
    int out_w, int out_h,
    cudaStream_t stream)
{
    int scaled_w{}, scaled_h{}, pad_left{}, pad_top{};
    preprocess_cuda_letterbox_params(
        src_w, src_h, out_w, out_h,
        scaled_w, scaled_h, pad_left, pad_top);

    // inv_sx / inv_sy: map a pixel in the scaled image back to source space.
    // For scaled_w==1 degenerate case avoid division by zero.
    const float inv_sx = scaled_w > 1
        ? static_cast<float>(src_w - 1) / static_cast<float>(scaled_w - 1)
        : 0.0f;
    const float inv_sy = scaled_h > 1
        ? static_cast<float>(src_h - 1) / static_cast<float>(scaled_h - 1)
        : 0.0f;

    constexpr int kTile = 32;
    const dim3 block(kTile, kTile);
    const dim3 grid(
        (static_cast<unsigned>(out_w) + kTile - 1) / kTile,
        (static_cast<unsigned>(out_h) + kTile - 1) / kTile);

    k_bgr_letterbox_to_rgb_chw<<<grid, block, 0, stream>>>(
        src_bgr, src_w, src_h,
        dst_chw, out_w, out_h,
        scaled_w, scaled_h, pad_left, pad_top,
        inv_sx, inv_sy);

    return static_cast<int>(cudaGetLastError());
}

} // namespace rmcs_laser_guidance

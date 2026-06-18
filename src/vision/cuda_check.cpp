#include "vision/cuda_check.hpp"

#include <cstdio>

#include <cuda_runtime_api.h>

namespace rmcs_laser_guidance {

auto cuda_device_available() noexcept -> bool {
    const auto err = cudaFree(nullptr);
    if (err == cudaSuccess) {
        int count = 0;
        if (cudaGetDeviceCount(&count) == cudaSuccess && count > 0)
            return true;
    }
    // Stub or driver missing — CUDA not functional
    std::fprintf(stderr, "CUDA: no device available (cudaFree returned %d)\n",
                 static_cast<int>(err));
    return false;
}

} // namespace rmcs_laser_guidance

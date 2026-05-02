#pragma once

#include <cstddef>
#include <cstdlib>
#include <cstring>

extern "C" {

using cudaError_t    = int;
using cudaStream_t   = void*;
using cudaEvent_t    = void*;
using cudaMemcpyKind = int;

constexpr cudaError_t cudaSuccess = 0;
constexpr cudaMemcpyKind cudaMemcpyHostToHost   = 0;
constexpr cudaMemcpyKind cudaMemcpyHostToDevice = 1;
constexpr cudaMemcpyKind cudaMemcpyDeviceToHost = 2;
constexpr cudaMemcpyKind cudaMemcpyDeviceToDevice = 3;

struct cudaDeviceProp {
    char name[256] { };
};

static inline cudaError_t cudaStreamCreate(cudaStream_t* stream) {
    if (stream == nullptr) return 1;
    *stream = reinterpret_cast<void*>(0x1);
    return cudaSuccess;
}

static inline cudaError_t cudaStreamDestroy(cudaStream_t) {
    return cudaSuccess;
}

static inline cudaError_t cudaStreamSynchronize(cudaStream_t) {
    return cudaSuccess;
}

static inline cudaError_t cudaMalloc(void** devPtr, std::size_t size) {
    if (devPtr == nullptr) return 1;
    *devPtr = std::malloc(size == 0 ? 1 : size);
    return *devPtr == nullptr ? 2 : cudaSuccess;
}

static inline cudaError_t cudaFree(void* devPtr) {
    std::free(devPtr);
    return cudaSuccess;
}

static inline cudaError_t cudaMemcpyAsync(
    void* dst, const void* src, std::size_t count, cudaMemcpyKind, cudaStream_t) {
    if (dst == nullptr || src == nullptr) return 1;
    std::memcpy(dst, src, count);
    return cudaSuccess;
}

static inline const char* cudaGetErrorString(cudaError_t error) {
    switch (error) {
    case cudaSuccess:
        return "cudaSuccess";
    case 1:
        return "cudaInvalidValue";
    case 2:
        return "cudaMemoryAllocation";
    default:
        return "cudaError";
    }
}

} // extern "C"

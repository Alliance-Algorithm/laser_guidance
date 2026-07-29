#pragma once

#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <vector>

namespace rmcs_laser_guidance {

struct TensorRTMeta;

class TensorRTEngine {
public:
    TensorRTEngine() = default;
    ~TensorRTEngine();

    TensorRTEngine(const TensorRTEngine&) = delete;
    auto operator=(const TensorRTEngine&) -> TensorRTEngine& = delete;
    TensorRTEngine(TensorRTEngine&&) noexcept;
    auto operator=(TensorRTEngine&&) noexcept -> TensorRTEngine&;

    static auto load(const std::string& path) -> std::expected<TensorRTEngine, std::string>;

    // Original path: accepts a pre-built float32 host buffer.
    auto run(
        const std::vector<float>& input,
        const std::vector<std::int64_t>& input_shape,
        std::vector<float>& output,
        std::vector<std::int64_t>& output_shape)
        -> std::expected<void, std::string>;

    // Fast path: BGR8 HWC host image → GPU preprocess kernel → TRT inference.
    // Eliminates the CPU preprocess_blob step (~18ms) and replaces the large
    // float H2D transfer with a smaller uint8 upload.
    // src_bgr : host pointer, uint8 BGR packed HWC [src_h, src_w, 3]
    // out_h/w : network input resolution (e.g. 1216)
    auto run_from_bgr(
        const unsigned char* src_bgr,
        int src_w, int src_h,
        int out_w, int out_h,
        std::vector<float>& output,
        std::vector<std::int64_t>& output_shape)
        -> std::expected<void, std::string>;

    auto meta() const -> const TensorRTMeta&;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_{};
};

struct TensorRTMeta {
    std::string engine_path{};
    std::string device_name{};

    struct TensorInfo {
        std::string name{};
        std::vector<std::int64_t> shape{};
        std::vector<std::int64_t> min_shape{};
        std::vector<std::int64_t> opt_shape{};
        std::vector<std::int64_t> max_shape{};
    };

    std::vector<TensorInfo> inputs{};
    std::vector<TensorInfo> outputs{};
};

} // namespace rmcs_laser_guidance

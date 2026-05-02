#pragma once

#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <vector>

namespace rmcs_laser_guidance {

struct TensorRTMeta {
    std::string engine_path { };
    std::string device_name { };

    struct TensorInfo {
        std::string name { };
        std::vector<std::int64_t> shape { };
    };

    std::vector<TensorInfo> inputs { };
    std::vector<TensorInfo> outputs { };
};

class TensorRTEngine {
public:
    TensorRTEngine() = default;
    ~TensorRTEngine();

    TensorRTEngine(const TensorRTEngine&)            = delete;
    auto operator=(const TensorRTEngine&) -> TensorRTEngine& = delete;
    TensorRTEngine(TensorRTEngine&&) noexcept;
    auto operator=(TensorRTEngine&&) noexcept -> TensorRTEngine&;

    static auto load(const std::string& path) -> std::expected<TensorRTEngine, std::string>;
    auto run(const std::vector<float>& input, std::vector<float>& output)
        -> std::expected<void, std::string>;
    auto meta() const -> const TensorRTMeta&;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_ { };
};

} // namespace rmcs_laser_guidance

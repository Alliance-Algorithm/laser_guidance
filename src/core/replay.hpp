#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

#include "types.hpp"

namespace rmcs_laser_guidance {

struct ReplayFrameInfo {
    std::size_t index = 0;
    std::int64_t timestamp_ns = 0;
    std::filesystem::path relative_image_path{};
};

struct ReplayDataset {
    std::filesystem::path root{};
    std::vector<ReplayFrameInfo> frames{};
};

class ReplayRecorder {
public:
    explicit ReplayRecorder(std::filesystem::path root);

    auto record_frame(const Frame& frame) -> ReplayFrameInfo;
    auto flush_manifest() const -> void;

    [[nodiscard]] auto root() const noexcept -> const std::filesystem::path& { return root_; }

private:
    std::filesystem::path root_;
    std::vector<ReplayFrameInfo> frames_;
};

auto timestamp_to_nanoseconds(const Clock::time_point& timestamp) -> std::int64_t;
auto timestamp_from_nanoseconds(std::int64_t nanoseconds) -> Clock::time_point;

auto load_replay_dataset(const std::filesystem::path& root) -> ReplayDataset;
auto load_replay_frame(const ReplayDataset& dataset, const ReplayFrameInfo& frame_info) -> Frame;

} // namespace rmcs_laser_guidance

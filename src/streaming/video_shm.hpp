#pragma once
#include <atomic>
#include <cstdint>
#include <cstddef>

namespace rmcs_laser_guidance {

constexpr const char* kShmName = "/laser_frame";
constexpr std::uint32_t kShmMagic = 0x4C465248; // "LFRH"
constexpr std::size_t kHeaderSize = 64;

struct ShmHeader {
    std::uint32_t magic;
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t stride;
    std::atomic<std::uint32_t> frame_seq;
    std::atomic<std::uint32_t> write_idx; // 0 or 1
    std::uint8_t _pad[40];
};
static_assert(sizeof(ShmHeader) == kHeaderSize);

class VideoShmProducer {
public:
    bool open(int width, int height);
    void push_frame(const std::uint8_t* bgr_data, int width, int height);
    void close();
    ~VideoShmProducer() { close(); }

private:
    int fd_ = -1;
    void* map_ = nullptr;
    std::size_t map_size_ = 0;
    ShmHeader* header_ = nullptr;
    std::uint8_t* buf_[2] = {};
};

} // namespace rmcs_laser_guidance

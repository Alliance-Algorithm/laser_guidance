#include "internal/video_shm.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <print>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace rmcs_laser_guidance {

bool VideoShmProducer::open(int width, int height) {
    const std::size_t frame_size =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3;
    map_size_ = kHeaderSize + 2 * frame_size;

    fd_ = shm_open(kShmName, O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
        std::println(stderr, "[shm] shm_open failed: {}", std::strerror(errno));
        return false;
    }

    if (ftruncate(fd_, static_cast<off_t>(map_size_)) < 0) {
        std::println(stderr, "[shm] ftruncate failed: {}", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    map_ = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (map_ == MAP_FAILED) {
        std::println(stderr, "[shm] mmap failed: {}", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        map_ = nullptr;
        return false;
    }

    header_ = static_cast<ShmHeader*>(map_);
    auto* base = static_cast<std::uint8_t*>(map_);
    buf_[0] = base + kHeaderSize;
    buf_[1] = base + kHeaderSize + frame_size;

    // Initialize header
    header_->magic = kShmMagic;
    header_->width = static_cast<std::uint32_t>(width);
    header_->height = static_cast<std::uint32_t>(height);
    header_->stride = static_cast<std::uint32_t>(width) * 3;
    header_->frame_seq.store(0, std::memory_order_relaxed);
    header_->write_idx.store(0, std::memory_order_relaxed);

    std::println("[shm] created {} ({}x{}, {:.1f} MB)",
                 kShmName, width, height,
                 static_cast<double>(map_size_) / (1024.0 * 1024.0));
    return true;
}

void VideoShmProducer::push_frame(const std::uint8_t* bgr_data, int width, int height) {
    if (!header_) return;

    const std::uint32_t idx = header_->write_idx.load(std::memory_order_relaxed);
    const std::size_t frame_size =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3;

    std::memcpy(buf_[idx], bgr_data, frame_size);

    // Flip write_idx first, then increment frame_seq (release fence)
    header_->write_idx.store(1 - idx, std::memory_order_relaxed);
    header_->frame_seq.fetch_add(1, std::memory_order_release);
}

void VideoShmProducer::close() {
    if (map_ && map_ != MAP_FAILED) {
        munmap(map_, map_size_);
        map_ = nullptr;
        header_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    shm_unlink(kShmName);
}

} // namespace rmcs_laser_guidance

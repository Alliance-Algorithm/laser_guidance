#pragma once

#include <memory>
#include <stdexcept>

#include "capture/capture_device.hpp"
#include "capture/v4l2_capture_device.hpp"
#include "capture/hik_camera_capture_device.hpp"
#include "config.hpp"

namespace rmcs_laser_guidance {

inline auto create_capture_device(const Config& config) -> std::unique_ptr<ICaptureDevice> {
    switch (config.capture_backend) {
    case CaptureBackendKind::v4l2:
        return std::make_unique<V4l2CaptureDevice>(config.v4l2);
    case CaptureBackendKind::hikcamera:
#ifdef WITH_HIKCAMERA
        return std::make_unique<HikCameraCaptureDevice>(
            HikCameraCaptureConfig{
                .device_id = config.hik.device_id,
                .timeout_ms = config.hik.timeout_ms,
                .exposure_us = config.hik.exposure_us,
                .framerate = config.hik.framerate,
                .gain = config.hik.gain,
                .invert_image = config.hik.invert_image,
                .software_sync = config.hik.software_sync,
                .trigger_mode = config.hik.trigger_mode,
                .fixed_framerate = config.hik.fixed_framerate,
            });
#else
        throw std::runtime_error("HikCamera backend requested but WITH_HIKCAMERA=OFF");
#endif
    }
    return nullptr;
}

} // namespace rmcs_laser_guidance

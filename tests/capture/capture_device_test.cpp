#include <print>
#include <stdexcept>
#include <vector>

#include <hikcamera/capturer.hpp>

#include "capture/capture_device.hpp"
#include "capture/hik_backend.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using rmcs_laser_guidance::CaptureBackendKind;
        using rmcs_laser_guidance::CaptureDevice;
        using rmcs_laser_guidance::CaptureFormat;
        using rmcs_laser_guidance::V4l2NegotiatedFormat;
        using rmcs_laser_guidance::tests::require;
        using rmcs_laser_guidance::tests::require_near;

        {
            const V4l2NegotiatedFormat source{
                .device_path = "/dev/video3",
                .width = 1280,
                .height = 720,
                .framerate = 59.94,
                .fourcc = "MJPG",
            };
            const CaptureFormat format = rmcs_laser_guidance::to_capture_format(source);
            require(format.backend == CaptureBackendKind::v4l2, "v4l2 backend mismatch");
            require(format.device_id == "/dev/video3", "v4l2 device id mismatch");
            require(format.width == 1280, "v4l2 width mismatch");
            require(format.height == 720, "v4l2 height mismatch");
            require_near(static_cast<float>(format.framerate), 59.94F, 1e-3F, "v4l2 framerate");
            require(format.format_name == "MJPG", "v4l2 format name mismatch");
        }

        {
            const hikcamera::DeviceInfo device_info{
                .device_id = "cam-a",
                .user_defined_name = "cam-a",
                .serial_number = "SN001",
                .model_name = "MV-CS016-10UC",
                .transport_layer = "USB",
            };
            const hikcamera::StreamFormat stream_format{
                .width = 1440,
                .height = 1080,
                .framerate = 119.88,
                .pixel_format_name = "BGR8",
                .source_pixel_format_name = "BayerRG8",
            };
            const CaptureFormat format =
                rmcs_laser_guidance::to_capture_format(device_info, stream_format);
            require(format.backend == CaptureBackendKind::hikcamera, "hik backend mismatch");
            require(format.device_id == "cam-a", "hik device id mismatch");
            require(format.width == 1440, "hik width mismatch");
            require(format.height == 1080, "hik height mismatch");
            require_near(static_cast<float>(format.framerate), 119.88F, 1e-3F, "hik framerate");
            require(format.format_name == "BGR8", "hik format name mismatch");
            require(format.pixel_encoding == "BGR8", "hik pixel encoding mismatch");
        }

        {
            rmcs_laser_guidance::HikCameraConfig config;
            config.exposure_us = 500.0F;
            config.framerate = 80.0F;
            config.gain = 10.0F;
            config.has_unlit_profile = true;
            config.unlit.exposure_us = 1000.0F;
            config.unlit.framerate = 50.0F;
            config.unlit.gain = 10.0F;
            config.unlit.set_white_balance = true;
            config.unlit.white_balance_ratio_red = 1450;
            config.unlit.white_balance_ratio_green = 900;
            config.unlit.white_balance_ratio_blue = 2300;

            const auto lit =
                rmcs_laser_guidance::select_startup_profile(config, rmcs_laser_guidance::HikProfileKind::lit);
            require(lit.has_value(), "lit profile selection should succeed");
            require_near(lit->exposure_us, 500.0F, 1e-3F, "lit profile exposure");
            require_near(lit->framerate, 80.0F, 1e-3F, "lit profile framerate");
            require(!lit->set_white_balance, "lit profile uses auto white balance");

            const auto unlit = rmcs_laser_guidance::select_startup_profile(
                config, rmcs_laser_guidance::HikProfileKind::unlit);
            require(unlit.has_value(), "unlit profile selection should succeed");
            require_near(unlit->exposure_us, 1000.0F, 1e-3F, "unlit profile exposure");
            require_near(unlit->framerate, 50.0F, 1e-3F, "unlit profile framerate");
            require(unlit->set_white_balance, "unlit profile uses manual white balance");
            require(unlit->white_balance_ratio_red == 1450, "unlit profile wb red");
        }

        {
            rmcs_laser_guidance::HikCameraConfig config;
            config.has_unlit_profile = false;
            const auto unlit = rmcs_laser_guidance::select_startup_profile(
                config, rmcs_laser_guidance::HikProfileKind::unlit);
            require(!unlit.has_value(), "unlit selection should fail without profile_unlit");
        }

        {
            const std::vector<hikcamera::DeviceInfo> devices{
                {
                    .device_id = "cam-a",
                    .user_defined_name = "front_cam",
                    .serial_number = "SN001",
                    .model_name = "MV-A",
                    .transport_layer = "USB",
                },
                {
                    .device_id = "cam-b",
                    .user_defined_name = "rear_cam",
                    .serial_number = "SN002",
                    .model_name = "MV-B",
                    .transport_layer = "GigE",
                },
            };

            const auto by_device_id = hikcamera::select_device_index(devices, "cam-b");
            require(by_device_id.has_value(), "device id selection should succeed");
            require(*by_device_id == 1, "device id selection index mismatch");

            const auto by_serial = hikcamera::select_device_index(devices, "SN001");
            require(by_serial.has_value(), "serial selection should succeed");
            require(*by_serial == 0, "serial selection index mismatch");

            const auto by_name = hikcamera::select_device_index(devices, "rear_cam");
            require(by_name.has_value(), "name selection should succeed");
            require(*by_name == 1, "name selection index mismatch");

            const auto missing = hikcamera::select_device_index(devices, "unknown");
            require(!missing.has_value(), "missing device selection should fail");

            const std::vector<hikcamera::DeviceInfo> single_device{
                {
                    .device_id = "only-cam",
                    .user_defined_name = "only-cam",
                    .serial_number = "SN100",
                    .model_name = "MV-ONLY",
                    .transport_layer = "USB",
                },
            };
            const auto single = hikcamera::select_device_index(single_device, "");
            require(single.has_value(), "single-device empty selection should succeed");
            require(*single == 0, "single-device selection index mismatch");
        }

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "capture_device_test failed: {}", e.what());
        return 1;
    }
}

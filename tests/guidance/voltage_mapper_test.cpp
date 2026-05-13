#include <cassert>
#include <cmath>
#include <print>

#include "config.hpp"
#include "guidance/voltage_mapper.hpp"
#include "test_utils.hpp"

int main() {
    using namespace rmcs_laser_guidance;
    using namespace rmcs_laser_guidance::tests;

    const auto path = make_temp_path("voltage_mapper_test");
    write_text_file(path,
        "model:\n"
        "  type: lut\n"
        "  u_axis: [0.0, 1.0]\n"
        "  v_axis: [0.0, 1.0]\n"
        "  log_area_axis: [0.0]\n"
        "  vx_values:\n"
        "    -\n"
        "      - [-1.0, 1.0]\n"
        "      - [-1.0, 1.0]\n"
        "  vy_values:\n"
        "    -\n"
        "      - [-1.0, -1.0]\n"
        "      - [1.0, 1.0]\n");

    GuidanceConfig cfg;
    cfg.command_model = GuidanceCommandModelKind::direct_voltage;
    cfg.voltage_model_path = path;
    cfg.voltage_limit_v = 5.0F;
    VoltageMapper mapper(cfg);

    const auto result = mapper.predict({
        .center_x = 50.0F,
        .center_y = 50.0F,
        .bbox_w = 20.0F,
        .bbox_h = 10.0F,
        .bbox_area = 200.0F,
        .score = 0.9F,
        .class_id = 1,
        .image_width = 100.0F,
        .image_height = 100.0F,
    });
    assert(result.has_value());
    assert(result->valid);
    assert(std::abs(result->vx) < 1e-4F);
    assert(std::abs(result->vy) < 1e-4F);
    std::println("voltage_mapper_test: PASSED");
    return 0;
}

#include <print>

#include "streaming/zmq_sender.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance;
        using namespace rmcs_laser_guidance::tests;

        const TargetObservation populated{
            .detected = true,
            .center = {12.25F, 34.5F},
            .contour = {{100.0F, 50.0F}, {200.0F, 50.0F}, {200.0F, 150.0F}, {100.0F, 150.0F}},
            .brightness = 0.5F,
            .candidates = {
                ModelCandidate{
                    .score = 0.75F,
                    .class_id = 0,
                    .bbox = {100.0F, 50.0F, 200.0F, 100.0F},
                    .center = {200.0F, 100.0F},
                },
            },
        };

        const std::string populated_json = serialize_laser_json(populated);
        require_contains(populated_json, "\"cmd_id\":8195", "cmd_id field");
        require_contains(populated_json, "\"detected\":true", "detected field");
        require_contains(populated_json, "\"center\":[12.25,34.5]", "center field");
        require_contains(populated_json, "\"brightness\":0.5", "brightness field");
        require_contains(
            populated_json,
            "\"contour\":[[100.0,50.0],[200.0,50.0],[200.0,150.0],[100.0,150.0]]",
            "contour field");
        require_contains(populated_json, "\"candidates\":[{", "candidates field");
        require_contains(populated_json, "\"score\":0.75", "candidate score field");
        require_contains(populated_json, "\"class_id\":0", "candidate class field");
        require_contains(populated_json, "\"bbox\":[100.0,50.0,200.0,100.0]", "candidate bbox field");
        require_contains(populated_json, "\"center\":[200.0,100.0]", "candidate center field");

        const TargetObservation empty{
            .detected = false,
            .center = {-1.0F, -1.0F},
            .contour = {},
            .brightness = 0.0F,
            .candidates = {},
        };

        const std::string empty_json = serialize_laser_json(empty);
        require_contains(empty_json, "\"detected\":false", "empty detected field");
        require_contains(empty_json, "\"center\":[-1.0,-1.0]", "empty center field");
        require_contains(empty_json, "\"contour\":[]", "empty contour field");
        require_contains(empty_json, "\"candidates\":[]", "empty candidates field");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "zmq_json_test failed: {}", e.what());
        return 1;
    }
}

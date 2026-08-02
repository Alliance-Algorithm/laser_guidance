#include "bridges/ros_bridge.hpp"

#include "laser_guidance/runtime.hpp"



#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace rmcs_laser_guidance {


namespace {

constexpr auto kTopicPrefix = "/laser_guidance/";
constexpr auto kFrameId = "laser_frame";

auto make_publisher(rclcpp::Node& node, const std::string& topic_suffix, int qos_depth = 10)
    -> rclcpp::PublisherBase::SharedPtr;

auto make_marker_header(
    const std::string& ns, int32_t id, const std::string& frame_id = kFrameId)
    -> std_msgs::msg::Header;

} // namespace

struct RosBridge::Impl {
    std::shared_ptr<rclcpp::Node> node;
    bool owns_ros_init = false;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detections_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aim_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hit_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr referee_pub;

    bool initialized = false;

    auto init() -> bool {
        try {
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
                owns_ros_init = true;
            }

            node = std::make_shared<rclcpp::Node>(
                "laser_guidance_ros_bridge",
                rclcpp::NodeOptions{}
                    .automatically_declare_parameters_from_overrides(true));

            detections_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
                std::string(kTopicPrefix) + "detections", 10);
            tracks_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
                std::string(kTopicPrefix) + "tracks", 10);
            aim_pub = node->create_publisher<visualization_msgs::msg::Marker>(
                std::string(kTopicPrefix) + "aim_point", 10);
            hit_pub = node->create_publisher<std_msgs::msg::Float64>(
                std::string(kTopicPrefix) + "hit_progress", 10);
            referee_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
                std::string(kTopicPrefix) + "referee", 10);

            initialized = true;
            RCLCPP_INFO(node->get_logger(), "ROS2 bridge initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("laser_guidance"), "ROS2 bridge init failed: %s", e.what());
            return false;
        }
    }

    auto publish_detections(const DetectionBatch& batch) -> void {
        if (!initialized || !detections_pub) return;
        if (batch.detections.empty()) return;

        visualization_msgs::msg::MarkerArray msg;
        int32_t id = 0;

        for (const auto& det : batch.detections) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = kFrameId;
            m.header.stamp = node->now();
            m.ns = "detections";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = det.center.x;
            m.pose.position.y = det.center.y;
            m.pose.position.z = 0.0;
            m.scale.x = det.bbox.width;
            m.scale.y = det.bbox.height;
            m.scale.z = 0.1;
            m.color.r = 0.0F;
            m.color.g = 1.0F;
            m.color.b = 0.0F;
            m.color.a = 0.7F;
            m.lifetime = rclcpp::Duration::from_seconds(0.2);
            msg.markers.push_back(m);
        }

        // Mark stale detections for deletion
        for (int32_t i = id; i < 50; ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = kFrameId;
            m.header.stamp = node->now();
            m.ns = "detections";
            m.id = i;
            m.action = visualization_msgs::msg::Marker::DELETE;
            msg.markers.push_back(m);
        }

        detections_pub->publish(msg);
    }

    auto publish_track(const TargetTrack& track) -> void {
        if (!initialized || !tracks_pub || !track.detected) return;

        visualization_msgs::msg::MarkerArray msg;

        // Track position sphere
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = kFrameId;
            m.header.stamp = node->now();
            m.ns = "tracks";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = track.aim_center.x;
            m.pose.position.y = track.aim_center.y;
            m.pose.position.z = 0.0;
            m.scale.x = 10.0;
            m.scale.y = 10.0;
            m.scale.z = 10.0;
            m.color.r = track.ekf_enabled ? 0.0F : 1.0F;
            m.color.g = track.ekf_enabled ? 1.0F : 0.5F;
            m.color.b = 1.0F;
            m.color.a = 0.8F;
            m.lifetime = rclcpp::Duration::from_seconds(0.2);
            msg.markers.push_back(m);
        }

        // Velocity arrow
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = kFrameId;
            m.header.stamp = node->now();
            m.ns = "tracks";
            m.id = 1;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = track.aim_center.x;
            m.pose.position.y = track.aim_center.y;
            m.pose.position.z = 0.0;
            m.scale.x = static_cast<double>(std::hypot(track.velocity.x, track.velocity.y));
            m.scale.y = 3.0;
            m.scale.z = 3.0;
            m.color.r = 1.0F;
            m.color.g = 0.8F;
            m.color.b = 0.0F;
            m.color.a = 0.6F;
            m.lifetime = rclcpp::Duration::from_seconds(0.2);
            // Direction from velocity
            double angle = std::atan2(track.velocity.y, track.velocity.x);
            m.pose.orientation.z = std::sin(angle / 2.0);
            m.pose.orientation.w = std::cos(angle / 2.0);
            msg.markers.push_back(m);
        }

        tracks_pub->publish(msg);
    }

    auto publish_aim(const AimOutput& aim) -> void {
        if (!initialized || !aim_pub || !aim.command_issued) return;

        visualization_msgs::msg::Marker m;
        m.header.frame_id = kFrameId;
        m.header.stamp = node->now();
        m.ns = "aim";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.5;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        m.color.r = 1.0F;
        m.color.g = 0.2F;
        m.color.b = 0.2F;
        m.color.a = 1.0F;
        m.lifetime = rclcpp::Duration::from_seconds(0.2);

        if (aim.output_angles) {
            m.pose.position.x = aim.output_angles->x;
            m.pose.position.y = aim.output_angles->y;
        }
        aim_pub->publish(m);
    }

    auto publish_hit(float progress) -> void {
        if (!initialized || !hit_pub) return;

        std_msgs::msg::Float64 msg;
        msg.data = static_cast<double>(progress);
        hit_pub->publish(msg);
    }

    auto publish_referee(const RefereeSnapshot& referee) -> void {
        if (!initialized || !referee_pub) return;
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {
            static_cast<double>(referee.game_progress),
            static_cast<double>(referee.match_elapsed_s),
            referee.official_aerial_targeted ? 1.0 : 0.0,
            referee.official_aerial_countered ? 1.0 : 0.0,
        };
        referee_pub->publish(msg);
    }

    auto spin() -> void {
        if (initialized && node) {
            rclcpp::spin_some(node);
        }
    }
};

RosBridge::RosBridge()
    : impl_(std::make_unique<Impl>()) {
    impl_->init();
}

RosBridge::~RosBridge() {
    if (impl_->owns_ros_init && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

auto RosBridge::ready() const noexcept -> bool { return impl_->initialized; }

auto RosBridge::publish_snapshot(const RuntimeSnapshot& snapshot) -> void {
    impl_->publish_detections(snapshot.detection);
    if (snapshot.track) {
        impl_->publish_track(*snapshot.track);
    }
    impl_->publish_aim(snapshot.aim);
    impl_->publish_hit(snapshot.hit_progress.progress);
    impl_->publish_referee(snapshot.referee);
}

auto RosBridge::spin() -> void { impl_->spin(); }

} // namespace rmcs_laser_guidance

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "System.h"          // kept here to avoid surprises at link time
#include <algorithm>         // std::clamp (used in the .cpp via this header)
#include <cmath>             // std::fabs  (same deal)

namespace fost {

/**
 * @brief Thin ROS 2 wrapper around the processing System.
 *        Publishes progress (latched-ish QoS) so late subscribers still get the latest tick.
 */
class SystemNode : public rclcpp::Node {
public:
    /// Construct with optional NodeOptions (namespaces, remaps, etc.).
    explicit SystemNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // --- Pub/Sub + worker bits ---
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;  // progress topic (kept small and reliable)
    std::shared_ptr<System> system_;                                      // the actual number cruncher

    // --- Progress smoothing (don’t spam the bus, mate) ---
    float        last_progress_ = -1.0f;  // hysteresis anchor
    rclcpp::Time last_pub_time_;          // quick rate-limit (~10 Hz in the .cpp)
};

} // namespace fost

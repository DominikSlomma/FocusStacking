#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "System.h"

namespace fost {

class SystemNode : public rclcpp::Node {
public:
    SystemNode();

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
    std::shared_ptr<System> system_;
};

}  // namespace fost


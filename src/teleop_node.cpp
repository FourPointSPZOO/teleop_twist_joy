// Copyright Haul Vision 2023

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "teleop_twist_joy/teleop_twist_joy.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<teleop_twist_joy::TeleopTwistJoy>(rclcpp::NodeOptions());

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}

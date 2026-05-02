#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robust_fast_navigation/planner_ros2.h"

int main(int argc, char ** argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerROS>();

    try {
        node->spin();
    }
    catch (const std::exception & e) {
        RCLCPP_FATAL(node->get_logger(), "Planner Node caught an exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

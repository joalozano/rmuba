#include "omni_position_controller.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<OmniPositionController>();

    rclcpp::spin(node);

    node->stop_timer();

    rclcpp::shutdown();

    return 0;
}
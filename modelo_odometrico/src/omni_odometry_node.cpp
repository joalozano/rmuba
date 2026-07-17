#include <rclcpp/rclcpp.hpp>
#include "omni_odometry.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin((std::make_shared<robmovil::OmniOdometry>()));
  rclcpp::shutdown();
  return 0;
}

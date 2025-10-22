#include <rclcpp/rclcpp.hpp>
#include "detect_landing_pad_behavior.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectLandingPadBehavior>());
  rclcpp::shutdown();
  return 0;
}

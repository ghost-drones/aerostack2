#include <rclcpp/rclcpp.hpp>
#include "detect_aruco_markers_behavior_pub_landingpad.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectArucoMarkersLandingPad>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

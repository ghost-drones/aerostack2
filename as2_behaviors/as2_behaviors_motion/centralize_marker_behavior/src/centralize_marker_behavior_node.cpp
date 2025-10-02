#include "as2_core/core_functions.hpp"
#include "centralize_marker_behavior/centralize_marker_behavior.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CentralizeMarkerBehavior>();
  node->preset_loop_frequency(30);
  as2::spinLoop(node);

  rclcpp::shutdown();
  return 0;
}
#include "as2_core/core_functions.hpp"
#include "precision_landing_behavior/precision_landing_behavior.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PrecisionLandingBehavior>();
  node->preset_loop_frequency(30);
  as2::spinLoop(node);

  rclcpp::shutdown();
  return 0;
}
#include "as2_behavior_tree/action/precision_landing_action.hpp"

namespace as2_behavior_tree
{
PrecisionLandingAction::PrecisionLandingAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<as2_msgs::action::PrecisionLanding>(
    xml_tag_name, as2_names::actions::behaviors::precisionlanding, conf) {}

void PrecisionLandingAction::on_tick() {getInput("marker_frame_id", goal_.marker_frame_id);}

void PrecisionLandingAction::on_wait_for_result(
  std::shared_ptr<const as2_msgs::action::PrecisionLanding::Feedback> feedback) {}

}  // namespace as2_behavior_tree

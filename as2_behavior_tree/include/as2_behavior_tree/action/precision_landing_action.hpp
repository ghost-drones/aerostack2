#ifndef AS2_BEHAVIOR_TREE__ACTION__PRECISION_LANDING_ACTION_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__PRECISION_LANDING_ACTION_HPP_

#include <string>
#include <memory>

#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/precision_landing.hpp"

#include "as2_behavior_tree/bt_action_node.hpp"

namespace as2_behavior_tree
{
class PrecisionLandingAction
  : public as2_behavior_tree::BtActionNode<as2_msgs::action::PrecisionLanding>
{
public:
  PrecisionLandingAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  void on_wait_for_result(
    std::shared_ptr<const as2_msgs::action::PrecisionLanding::Feedback> feedback);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<std::string>("marker_frame_id")});
  }
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__PRECISION_LANDING_ACTION_HPP_

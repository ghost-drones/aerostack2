#include "as2_behavior_tree/condition/is_aruco_detected_condition.hpp"

namespace as2_behavior_tree
{
IsArucoDetectedCondition::IsArucoDetectedCondition(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  getInput("topic_name", topic_name_);
  getInput("dist_threshold", dist_threshold_);
  getInput("aruco_id", aruco_id_);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  aruco_sub_ = node_->create_subscription<as2_msgs::msg::PoseStampedWithIDArray>(
    topic_name_, 100,
    std::bind(&IsArucoDetectedCondition::arucoCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsArucoDetectedCondition::tick()
{
  callback_group_executor_.spin_some();
  if (aruco_detected_) {
    setOutput("pose", aruco_pose_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace as2_behavior_tree

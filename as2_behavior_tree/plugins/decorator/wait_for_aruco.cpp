#include "as2_behavior_tree/decorator/wait_for_aruco.hpp"

namespace as2_behavior_tree
{
WaitForAruco::WaitForAruco(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  getInput("topic_name", topic_name_);
  getInput("aruco_id", aruco_id_);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  sub_ = node_->create_subscription<as2_msgs::msg::PoseStampedWithIDArray>(
    topic_name_, rclcpp::SystemDefaultsQoS(),
    std::bind(&WaitForAruco::callback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus WaitForAruco::tick()
{
  getInput("topic_name", topic_name_);
  getInput("aruco_id", aruco_id_);
  
  callback_group_executor_.spin_some();
  if (flag_) {
    return child_node_->executeTick();
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForAruco::callback(const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg)
{
  if (!msg->poses.empty())
  {
    for (const auto &pose_with_id : msg->poses)
    {
      if (pose_with_id.id == std::to_string(aruco_id_))
      {
        const auto &pos = pose_with_id.pose.pose.position;
        const auto &ori = pose_with_id.pose.pose.orientation;

        std::string pose_str = std::to_string(pos.x) + ";" +
                              std::to_string(pos.y) + ";" +
                              std::to_string(pos.z) + ";" +
                              std::to_string(ori.x) + ";" +
                              std::to_string(ori.y) + ";" +
                              std::to_string(ori.z) + ";" +
                              std::to_string(ori.w);

        setOutput("aruco_pose", pose_str);

        flag_ = true;
        return;
      }
    }
  }
}


}  // namespace as2_behavior_tree

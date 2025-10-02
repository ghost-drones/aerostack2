#include "as2_behavior_tree/decorator/wait_for_tf_decorator.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace as2_behavior_tree
{

using namespace std::chrono_literals;

WaitForTFDecorator::WaitForTFDecorator(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus WaitForTFDecorator::tick()
{
  std::string target_frame, source_frame;
  getInput("target_frame", target_frame);
  getInput("source_frame", source_frame);

  try {
    auto transform = tf_buffer_->lookupTransform(
      source_frame, target_frame, tf2::TimePointZero, 100ms);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    std::string position_str = std::to_string(pose.pose.position.x) + ";" +
                        std::to_string(pose.pose.position.y) + ";" +
                        std::to_string(pose.pose.position.z);

    std::string pose_str = std::to_string(pose.pose.position.x) + ";" +
                           std::to_string(pose.pose.position.y) + ";" +
                           std::to_string(pose.pose.position.z) + ";" +
                           std::to_string(pose.pose.orientation.x) + ";" +
                           std::to_string(pose.pose.orientation.y) + ";" +
                           std::to_string(pose.pose.orientation.z) + ";" +
                           std::to_string(pose.pose.orientation.w);

    setOutput("tf_pose", pose_str);
    setOutput("tf_position", position_str);
    flag_ = true;
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "TF not available: %s", ex.what());
    flag_ = false;
  }

  if (flag_) {
    return child_node_->executeTick();
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace as2_behavior_tree

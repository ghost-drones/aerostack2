#ifndef AS2_BEHAVIOR_TREE__DECORATOR__WAIT_FOR_ARUCO_HPP_
#define AS2_BEHAVIOR_TREE__DECORATOR__WAIT_FOR_ARUCO_HPP_

#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/pose_stamped_with_id_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace as2_behavior_tree
{
class WaitForAruco : public BT::DecoratorNode
{
public:
  WaitForAruco(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("topic_name"), BT::InputPort<int>("aruco_id"), BT::OutputPort("aruco_pose")};
  }

private:
  BT::NodeStatus tick() override;

private:
  void callback(as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr sub_;
  std::string topic_name_;
  int aruco_id_ = 0;
  bool flag_ = false;
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__DECORATOR__WAIT_FOR_ARUCO_HPP_

#ifndef AS2_BEHAVIOR_TREE__DECORATOR__WAIT_FOR_TF_DECORATOR_HPP_
#define AS2_BEHAVIOR_TREE__DECORATOR__WAIT_FOR_TF_DECORATOR_HPP_

#include <string>
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace as2_behavior_tree
{

class WaitForTFDecorator : public BT::DecoratorNode
{
public:
  WaitForTFDecorator(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame"),
      BT::InputPort<std::string>("source_frame"),
      BT::OutputPort<std::string>("tf_pose"),
      BT::OutputPort<std::string>("tf_position"),
    };
  }

private:
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool flag_ = false;
  geometry_msgs::msg::PoseStamped last_pose_;
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__DECORATOR__WAIT_FOR_TF_HPP_

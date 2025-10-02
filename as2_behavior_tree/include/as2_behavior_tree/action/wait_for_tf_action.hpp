#ifndef AS2_BEHAVIOR_TREE__ACTION__WAIT_FOR_TF_ACTION_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__WAIT_FOR_TF_ACTION_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace as2_behavior_tree
{

class WaitForTFAction : public BT::StatefulActionNode
{
public:
  WaitForTFAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame"),
      BT::InputPort<std::string>("source_frame"),
      BT::InputPort<double>("tf_timeout_threshold"),
      BT::OutputPort<std::string>("tf_pose")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Time start_time_;
  double timeout_threshold_;
  std::string target_frame_;
  std::string source_frame_;
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__WAIT_FOR_TF_ACTION_HPP_

//TODO: continue development

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "as2_behavior_tree/action/wait_for_tf_action.hpp"
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
    const BT::NodeConfiguration & conf)
  : BT::StatefulActionNode(xml_tag_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(),
      node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame"),
      BT::InputPort<std::string>("source_frame"),
      BT::InputPort<double>("tf_timeout_threshold", 3.0),  // default 3s
      BT::OutputPort<std::string>("tf_pose")
    };
  }

protected:
  BT::NodeStatus onStart() override
  {
    cancelled_ = false;   // reset cancel flag
    start_time_ = node_->get_clock()->now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (cancelled_) {
      RCLCPP_INFO(node_->get_logger(), "WaitForTFAction cancelled");
      return BT::NodeStatus::FAILURE;
    }

    std::string target_frame, source_frame;
    getInput("target_frame", target_frame);
    getInput("source_frame", source_frame);
    double timeout;
    getInput("tf_timeout_threshold", timeout);

    try {
      auto transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, 100ms);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;

      std::string pose_str = std::to_string(pose.pose.position.x) + ";" +
                             std::to_string(pose.pose.position.y) + ";" +
                             std::to_string(pose.pose.position.z) + ";" +
                             std::to_string(pose.pose.orientation.x) + ";" +
                             std::to_string(pose.pose.orientation.y) + ";" +
                             std::to_string(pose.pose.orientation.z) + ";" +
                             std::to_string(pose.pose.orientation.w);

      setOutput("tf_pose", pose_str);
      return BT::NodeStatus::SUCCESS;

    } catch (const tf2::TransformException & ex) {
      auto now = node_->get_clock()->now();
      if ((now - start_time_).seconds() > timeout) {
        RCLCPP_WARN(node_->get_logger(), "TF wait timeout exceeded");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "WaitForTFAction halted externally");
    cancelled_ = true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Time start_time_;
  bool cancelled_ = false;
};

}  // namespace as2_behavior_tree
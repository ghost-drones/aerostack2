#ifndef AS2_BEHAVIOR_TREE__CONDITION__IS_ARUCO_DETECTED_CONDITION_HPP_
#define AS2_BEHAVIOR_TREE__CONDITION__IS_ARUCO_DETECTED_CONDITION_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp_v3/condition_node.h"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/pose_stamped_with_id_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2_behavior_tree
{
class IsArucoDetectedCondition : public BT::ConditionNode
{
public:
  IsArucoDetectedCondition(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  IsArucoDetectedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("aruco_id"),
      BT::InputPort<std::string>("topic_name"),
      BT::InputPort<double>("dist_threshold"),
      BT::OutputPort<as2_msgs::msg::PoseStampedWithID>("pose")
    };
  }

private:
  void arucoCallback(as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg)
  {
    for (const auto & ps_id : msg->poses)
    {
        if (ps_id.id == std::to_string(aruco_id_)) {

          double dx = ps_id.pose.pose.position.x;
          double dy = ps_id.pose.pose.position.y;
          double dz = ps_id.pose.pose.position.z;

          double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

          if (distance <= dist_threshold_) {
            aruco_detected_ = true;
            aruco_pose_ = ps_id;
            RCLCPP_INFO(node_->get_logger(), "ArUco detectado! ID: %d, Distância: %.3f", aruco_id_, distance);
          } 
          else {
            aruco_detected_ = false;
            RCLCPP_DEBUG(node_->get_logger(), "ArUco ID: %d encontrado, mas fora do threshold: %.3f", aruco_id_, distance);
          }
        } 
        else {
          aruco_detected_ = false;
          RCLCPP_DEBUG(node_->get_logger(), "ArUco ignorado, ID: %s não corresponde ao esperado: %d", ps_id.id.c_str(), aruco_id_);
        }
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr aruco_sub_;

  int aruco_id_;
  std::string topic_name_;
  double dist_threshold_;
  bool aruco_detected_ = false;
  as2_msgs::msg::PoseStampedWithID aruco_pose_;
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__CONDITION__IS_ARUCO_DETECTED_CONDITION_HPP_

// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file wait_for_event.hpp
 *
 * Wait for event implementation as behavior tree node
 *
 * @authors Pedro Arias Pérez
 *          Rafael Perez-Segui
 *          Miguel Fernández Cortizas
 */

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

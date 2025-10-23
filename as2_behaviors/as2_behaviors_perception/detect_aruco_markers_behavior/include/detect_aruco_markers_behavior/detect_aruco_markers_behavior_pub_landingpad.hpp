#ifndef DETECT_ARUCO_MARKERS_BEHAVIOR_PUB_LANDINGPAD_HPP_
#define DETECT_ARUCO_MARKERS_BEHAVIOR_PUB_LANDINGPAD_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

class DetectArucoMarkersLandingPad : public rclcpp::Node
{
public:
  DetectArucoMarkersLandingPad();

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  void computeLandingPadAverage();
};

#endif  // DETECT_ARUCO_MARKERS_BEHAVIOR_PUB_LANDINGPAD_HPP_

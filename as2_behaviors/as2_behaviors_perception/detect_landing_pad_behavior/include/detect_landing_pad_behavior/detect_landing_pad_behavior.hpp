#ifndef DETECT_LANDING_PAD_BEHAVIOR__DETECT_LANDING_PAD_BEHAVIOR_HPP__
#define DETECT_LANDING_PAD_BEHAVIOR__DETECT_LANDING_PAD_BEHAVIOR_HPP__

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_msgs/action/detect_landing_pad.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/pose_stamped_with_id_array.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2_ros/transform_broadcaster.h>

class DetectLandingPadBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::DetectLandingPad>
{
public:
  /**
   * @brief Construct a new Aruco Detector object
   */
  DetectLandingPadBehavior();

  /**
   * @brief Destroy the Aruco Detector object
   */
  ~DetectLandingPadBehavior() {}

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr aruco_pose_pub_;
  std::shared_ptr<as2::sensors::Camera> aruco_img_transport_;

  std::vector<uint16_t> target_ids_;
  float aruco_size_default_;
  std::string camera_model_;
  std::string distorsion_model_;
  bool camera_qos_reliable_;
  bool camera_params_available_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  std::string img_encoding_;
  std::string camera_image_topic_ = "camera/image_raw";
  std::string camera_info_topic_ = "camera/camera_info";
  std::unordered_map<int, float> aruco_sizes_;

  void loadParameters();
  void setup();
  void setCameraParameters(const sensor_msgs::msg::CameraInfo & _camera_info);
  bool checkIdIsTarget(const int _id);

public:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr img);
  void camerainfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info);

private:
  /** As2 Behavior methods **/
  bool on_activate(std::shared_ptr<const as2_msgs::action::DetectLandingPad::Goal> goal) override;

  bool on_modify(std::shared_ptr<const as2_msgs::action::DetectLandingPad::Goal> goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::DetectLandingPad::Goal> & goal,
    std::shared_ptr<as2_msgs::action::DetectLandingPad::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::DetectLandingPad::Result> & result_msg) override;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;
};

std::string targetIds2string(const std::vector<uint16_t> & target_ids);

#endif  // DETECT_LANDING_PAD_BEHAVIOR__DETECT_LANDING_PAD_BEHAVIOR_HPP__

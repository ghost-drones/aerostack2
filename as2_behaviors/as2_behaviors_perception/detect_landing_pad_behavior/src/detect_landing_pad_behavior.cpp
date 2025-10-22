#include "detect_landing_pad_behavior.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <unordered_map>

DetectLandingPadBehavior::DetectLandingPadBehavior()
: as2_behavior::BehaviorServer<as2_msgs::action::DetectLandingPad>(
    "detect_landing_pad_behavior",
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
{
  loadParameters();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void DetectLandingPadBehavior::setup()
{
  aruco_pose_pub_ = this->create_publisher<as2_msgs::msg::PoseStampedWithIDArray>(
    this->generate_local_name("aruco_pose"), rclcpp::SensorDataQoS());
  aruco_img_transport_ = std::make_shared<as2::sensors::Camera>(this, "aruco_img_topic");

  std::shared_ptr<const rclcpp::QoS> camera_qos;
  if (camera_qos_reliable_) {
    RCLCPP_INFO(get_logger(), "QoS Camera subscription: Reliable");
    camera_qos = std::make_shared<const rclcpp::QoS>(rclcpp::QoS(2));
  } else {
    RCLCPP_INFO(get_logger(), "QoS Camera subscription: Sensor");
    camera_qos = std::make_shared<const rclcpp::QoS>(rclcpp::SensorDataQoS());
  }

  cam_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_image_topic_, *camera_qos,
    std::bind(&DetectLandingPadBehavior::imageCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, as2_names::topics::sensor_measurements::qos,
    std::bind(&DetectLandingPadBehavior::camerainfoCallback, this, std::placeholders::_1));
}

void DetectLandingPadBehavior::loadParameters()
{
  this->get_parameter("aruco_size_default", aruco_size_default_);
  this->get_parameter("camera_qos_reliable", camera_qos_reliable_);
  this->get_parameter("camera_image_topic", camera_image_topic_);
  this->get_parameter("camera_info_topic", camera_info_topic_);
  this->get_parameter("camera_model", camera_model_);

  auto node_params = this->list_parameters({}, 10);
  for (const auto &name : node_params.names) {
    if (name.rfind("aruco_sizes_", 0) == 0) {
      double size;
      if (this->get_parameter(name, size)) {
        std::string key = name.substr(std::string("aruco_sizes_").length());
        try {
          int id = std::stoi(key);
          aruco_sizes_[id] = static_cast<float>(size);
        } catch (const std::exception &) {
          RCLCPP_WARN(get_logger(), "Invalid key: %s", key.c_str());
        }
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu ArUco marker sizes", aruco_sizes_.size());
  RCLCPP_INFO(get_logger(), "Default size: %.3f m", aruco_size_default_);

  aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
}

void DetectLandingPadBehavior::setCameraParameters(
  const sensor_msgs::msg::CameraInfo & _camera_info)
{
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; i++) {
    camera_matrix_.at<double>(i / 3, i % 3) = _camera_info.k[i];
  }

  int n_discoeff = _camera_info.d.size();
  dist_coeffs_ = cv::Mat(1, n_discoeff, CV_64F);
  for (int i = 0; i < n_discoeff; i++) {
    dist_coeffs_.at<double>(0, i) = _camera_info.d[i];
  }

  distorsion_model_ = _camera_info.distortion_model;

  if (camera_model_ == "fisheye") {
    RCLCPP_INFO(get_logger(), "Using FISHEYE camera model");
    if (n_discoeff != 4) {
      RCLCPP_ERROR(get_logger(), "FISHEYE distortion coefficients must be 4");
    }
  }
  if (camera_model_ == "pinhole") {
    RCLCPP_INFO(get_logger(), "Using PINHOLE camera model");
  }

  img_encoding_ = sensor_msgs::image_encodings::BGR8;
  camera_params_available_ = true;
}

void DetectLandingPadBehavior::camerainfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  RCLCPP_DEBUG(this->get_logger(), "Camera info received by callback");
  if (camera_params_available_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Setting camera info");
  setCameraParameters(*info);
}

void DetectLandingPadBehavior::imageCallback(const sensor_msgs::msg::Image::SharedPtr img)
{
  RCLCPP_DEBUG(this->get_logger(), "Image received by callback");

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, img_encoding_);
  } catch (cv_bridge::Exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "CV_bridge exception: %s\n", ex.what());
    return;
  }

  if (!camera_params_available_) {
    RCLCPP_WARN(this->get_logger(), "No camera parameters available");
    return;
  }

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();

  cv::aruco::detectMarkers(
    cv_ptr->image, aruco_dict_, marker_corners, marker_ids, detector_params, rejected_candidates);

  cv::Mat output_image = cv_ptr->image.clone();

  if (marker_ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
    std::vector<cv::Vec3d> rvecs(marker_ids.size()), tvecs(marker_ids.size());

    for (size_t i = 0; i < marker_ids.size(); ++i) {
      int id = marker_ids[i];
      float marker_size = aruco_size_default_;
      auto it = aruco_sizes_.find(id);
      if (it != aruco_sizes_.end()) {
        marker_size = it->second;
      }

      std::vector<std::vector<cv::Point2f>> single_corner = {marker_corners[i]};
      std::vector<cv::Vec3d> rvec_tmp, tvec_tmp;
      cv::aruco::estimatePoseSingleMarkers(single_corner, marker_size, camera_matrix_, dist_coeffs_, rvec_tmp, tvec_tmp);

      rvecs[i] = rvec_tmp[0];
      tvecs[i] = tvec_tmp[0];
    }

    auto aruco_pose_array = as2_msgs::msg::PoseStampedWithIDArray();
    aruco_pose_array.poses.reserve(marker_ids.size());

    for (size_t i = 0; i < rvecs.size(); ++i) {
      int id = marker_ids[i];
      if (!checkIdIsTarget(id)) {
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Marker %d detected", id);
      const cv::Vec3d &rvec = rvecs[i];
      const cv::Vec3d &tvec = tvecs[i];

      cv::drawFrameAxes(output_image, camera_matrix_, dist_coeffs_, rvec, tvec, 0.08625, 3);

      as2_msgs::msg::PoseStampedWithID pose;
      pose.pose.header = img->header;
      pose.id = std::to_string(id);
      pose.pose.pose.position.x = tvec[0];
      pose.pose.pose.position.y = tvec[1];
      pose.pose.pose.position.z = tvec[2];

      tf2::Quaternion rot;
      rot.setRPY(rvec[2], rvec[0], rvec[1]);
      rot.normalize();

      pose.pose.pose.orientation.x = static_cast<double>(rot.x());
      pose.pose.pose.orientation.y = static_cast<double>(rot.y());
      pose.pose.pose.orientation.z = static_cast<double>(rot.z());
      pose.pose.pose.orientation.w = static_cast<double>(rot.w());
      aruco_pose_array.poses.push_back(pose);

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = img->header.stamp;
      tf_msg.header.frame_id = img->header.frame_id + "/optical_frame";
      tf_msg.child_frame_id = "aruco_" + std::to_string(id);
      tf_msg.transform.translation.x = pose.pose.pose.position.x;
      tf_msg.transform.translation.y = pose.pose.pose.position.y;
      tf_msg.transform.translation.z = pose.pose.pose.position.z;
      tf_msg.transform.rotation = pose.pose.pose.orientation;

      tf_broadcaster_->sendTransform(tf_msg);
      RCLCPP_DEBUG(this->get_logger(), "TF published for aruco_%d", id);
    }

    aruco_pose_pub_->publish(aruco_pose_array);
  }

  cv::Mat rectified_image;
  if (camera_model_ == "pinhole") {
    cv::undistort(output_image, rectified_image, camera_matrix_, dist_coeffs_);
  } else if (camera_model_ == "fisheye") {
    cv::fisheye::undistortImage(output_image, rectified_image, camera_matrix_, dist_coeffs_);
  } else {
    rectified_image = output_image.clone();
  }

  sensor_msgs::msg::Image output_image_msg =
    *(cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, rectified_image)
    .toImageMsg().get());
  aruco_img_transport_->updateData(output_image_msg);
}

bool DetectLandingPadBehavior::checkIdIsTarget(const int _id)
{
  if (!target_ids_.empty() &&
      std::find(target_ids_.begin(), target_ids_.end(), _id) == target_ids_.end()) {
    return false;
  }
  return true;
}

bool DetectLandingPadBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::DetectLandingPad::Goal> goal)
{
  setup();
  target_ids_ = goal->target_ids;
  RCLCPP_INFO(get_logger(), "Goal accepted");
  RCLCPP_INFO(get_logger(), "Target IDs: %s", targetIds2string(target_ids_).c_str());
  return true;
}

bool DetectLandingPadBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::DetectLandingPad::Goal> goal)
{
  target_ids_ = goal->target_ids;
  RCLCPP_INFO(get_logger(), "Goal modified");
  RCLCPP_INFO(get_logger(), "New target IDs: %s", targetIds2string(target_ids_).c_str());
  return true;
}

bool DetectLandingPadBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(get_logger(), "DetectLandingPadBehavior cancelled");
  return true;
}

bool DetectLandingPadBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(get_logger(), "DetectLandingPadBehavior paused");
  return true;
}

bool DetectLandingPadBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(get_logger(), "DetectLandingPadBehavior resumed");
  return true;
}

as2_behavior::ExecutionStatus DetectLandingPadBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::DetectLandingPad::Goal> & goal,
  std::shared_ptr<as2_msgs::action::DetectLandingPad::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::DetectLandingPad::Result> & result_msg)
{
  return as2_behavior::ExecutionStatus::RUNNING;
}

void DetectLandingPadBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  cam_image_sub_.reset();
  cam_info_sub_.reset();
  aruco_pose_pub_.reset();
  aruco_img_transport_.reset();
  target_ids_.clear();

  RCLCPP_INFO(get_logger(), "DetectLandingPadBehavior execution ended");
}

std::string targetIds2string(const std::vector<uint16_t> & target_ids)
{
  if (target_ids.empty()) return "All";
  std::string result;
  for (size_t i = 0; i < target_ids.size(); i++) {
    result += std::to_string(target_ids[i]);
    if (i < target_ids.size() - 1) result += ", ";
  }
  return result;
}

#include "detect_aruco_markers_behavior_pub_landingpad.hpp"
#include <tf2/exceptions.h>

DetectArucoMarkersLandingPad::DetectArucoMarkersLandingPad()
: Node("detect_aruco_markers_behavior_pub_landingpad")
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&DetectArucoMarkersLandingPad::computeLandingPadAverage, this));

  RCLCPP_INFO(this->get_logger(), "LandingPad publisher node started.");
}

void DetectArucoMarkersLandingPad::computeLandingPadAverage()
{
  std::vector<std::string> frame_ids;
  try {
    frame_ids = tf_buffer_->getAllFrameNames();
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF frames: %s", e.what());
    return;
  }

  std::vector<geometry_msgs::msg::TransformStamped> pad_transforms;
  for (const auto &frame : frame_ids) {
    if (frame.rfind("pad_", 0) != 0) continue;

    try {
      if (!tf_buffer_->canTransform("earth", frame, tf2::TimePointZero)) continue;
      auto tf = tf_buffer_->lookupTransform("earth", frame, tf2::TimePointZero);
      pad_transforms.push_back(tf);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_DEBUG(this->get_logger(), "TF %s not available: %s", frame.c_str(), ex.what());
    }
  }

  if (pad_transforms.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[LandingPad] Nenhum pad_ disponível no TF.");
    return;
  }

  // Média das posições
  Eigen::Vector3d pos_sum(0, 0, 0);
  std::vector<Eigen::Quaterniond> quats;
  for (const auto &tf : pad_transforms) {
    pos_sum += Eigen::Vector3d(tf.transform.translation.x,
                               tf.transform.translation.y,
                               tf.transform.translation.z);
    Eigen::Quaterniond q(tf.transform.rotation.w,
                         tf.transform.rotation.x,
                         tf.transform.rotation.y,
                         tf.transform.rotation.z);
    quats.push_back(q);
  }

  Eigen::Vector3d avg_pos = pos_sum / pad_transforms.size();

  // Média das orientações
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  for (const auto &q : quats) {
    Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
    M += v * v.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(M);
  Eigen::Vector4d avg_q_vec = es.eigenvectors().col(3);
  Eigen::Quaterniond avg_q(avg_q_vec(0), avg_q_vec(1), avg_q_vec(2), avg_q_vec(3));
  avg_q.normalize();

  geometry_msgs::msg::TransformStamped avg_tf;
  avg_tf.header.stamp = this->now();
  avg_tf.header.frame_id = "earth";
  avg_tf.child_frame_id = "landing_pad";
  avg_tf.transform.translation.x = avg_pos.x();
  avg_tf.transform.translation.y = avg_pos.y();
  avg_tf.transform.translation.z = avg_pos.z();
  avg_tf.transform.rotation.x = avg_q.x();
  avg_tf.transform.rotation.y = avg_q.y();
  avg_tf.transform.rotation.z = avg_q.z();
  avg_tf.transform.rotation.w = avg_q.w();

  tf_broadcaster_->sendTransform(avg_tf);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "[LandingPad] TF publicado: média de %zu pads -> (%.2f, %.2f, %.2f)",
                       pad_transforms.size(), avg_pos.x(), avg_pos.y(), avg_pos.z());
}

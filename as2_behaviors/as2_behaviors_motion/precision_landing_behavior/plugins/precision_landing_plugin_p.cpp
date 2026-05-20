// BSD-3-Clause
/**
 * @file precision_landing_plugin_p.cpp
 *
 * Plugin: proportional (P) XY control + proportional yaw + constant descent inside radius.
 *
 * When yaw_detached is true the run loop goes through two sequential alignment
 * stages before descent:
 *   ALIGNING_XY  – correct XY until dist_xy < yaw_detached_xy_threshold
 *   ALIGNING_YAW – correct yaw until |yaw_err| < yaw_detached_yaw_threshold
 * After both stages pass, normal XY+yaw+descent runs.
 */

#include <algorithm>
#include <cmath>
#include <tuple>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"
#include "precision_landing_behavior/precision_landing_base.hpp"

namespace precision_landing_plugin_p
{

static double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

class Plugin : public precision_landing_base::PrecisionLandingBase
{
public:
  void ownInit() override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[p] Init start");

    speed_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(node_ptr_);

    node_ptr_->declare_parameter<std::string>("marker_frame_id", "landing_pad");
    node_ptr_->get_parameter("marker_frame_id", marker_frame_id_);

    node_ptr_->declare_parameter<double>("p_xy_kp", 1.0);
    node_ptr_->get_parameter("p_xy_kp", xy_kp_);
    node_ptr_->declare_parameter<double>("p_yaw_kp", 1.0);
    node_ptr_->get_parameter("p_yaw_kp", yaw_kp_);

    // Shared parameters
    node_ptr_->declare_parameter<double>("z_descent", 0.3);
    node_ptr_->get_parameter("z_descent", z_descent_);
    node_ptr_->declare_parameter<double>("z_distance_threshold", 0.1);
    node_ptr_->get_parameter("z_distance_threshold", z_distance_threshold_);
    node_ptr_->declare_parameter<double>("xy_speed_max", 1.0);
    node_ptr_->get_parameter("xy_speed_max", xy_speed_max_);
    node_ptr_->declare_parameter<double>("landing_radius", 0.3);
    node_ptr_->get_parameter("landing_radius", landing_radius_);
    node_ptr_->declare_parameter<double>("yaw_speed_max", 0.5);
    node_ptr_->get_parameter("yaw_speed_max", yaw_speed_max_);

    node_ptr_->declare_parameter<bool>("yaw_detached", false);
    node_ptr_->get_parameter("yaw_detached", yaw_detached_);
    node_ptr_->declare_parameter<double>("yaw_detached_xy_threshold", 0.3);
    node_ptr_->get_parameter("yaw_detached_xy_threshold", yaw_detached_xy_threshold_);
    node_ptr_->declare_parameter<double>("yaw_detached_yaw_threshold", 0.1);
    node_ptr_->get_parameter("yaw_detached_yaw_threshold", yaw_detached_yaw_threshold_);

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "[p] xy_kp=%.2f xy_vmax=%.2f | z_descent=%.2f z_th=%.2f radius=%.2f"
        " | yaw_kp=%.2f yaw_vmax=%.2f | yaw_detached=%s | marker=%s",
        xy_kp_, xy_speed_max_, z_descent_, z_distance_threshold_, landing_radius_,
        yaw_kp_, yaw_speed_max_, yaw_detached_ ? "true" : "false",
        marker_frame_id_.c_str());

    resetStatus();
  }

  bool own_activate(as2_msgs::action::PrecisionLanding::Goal &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[p] Precision Landing accepted");
    resetStatus();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[p] Precision Landing canceled, hover");
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[p] Precision Landing paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[p] Precision Landing resumed");
    resetStatus();
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[p] end - state: %d", (int)state);
    if (state != as2_behavior::ExecutionStatus::SUCCESS)
      sendHover();
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    geometry_msgs::msg::TransformStamped tf_aruco;
    if (!tryGetArucoTF(tf_aruco)) {
      if (arucoTimeoutExceeded()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "[p] ArUco TF timeout -> failure");
        result_.precision_landing_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
      RCLCPP_INFO(node_ptr_->get_logger(), "[p] No ArUco TF yet, hovering...");
      sendHover();
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    const auto [dx, dy, dz] = computeRelativeError(tf_aruco);
    const double dist_xy    = std::hypot(dx, dy);
    const double abs_dz     = std::fabs(dz);

    if (abs_dz < z_distance_threshold_) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[p] z threshold reached (|dz|=%.3f). Success.", abs_dz);
      result_.precision_landing_success = true;
      sendHover();
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    const double pad_yaw   = static_cast<double>(
        as2::frame::getYawFromQuaternion(tf_aruco.transform.rotation));
    const double drone_yaw = static_cast<double>(getActualYaw());
    const double yaw_err   = normalizeAngle(pad_yaw - drone_yaw);

    double vx = 0.0, vy = 0.0, vz = 0.0, yaw_speed = 0.0;

    if (yaw_detached_ && state_ != AlignState::DESCENDING) {
      runAlignmentFSM(dx, dy, dist_xy, yaw_err, vx, vy, vz, yaw_speed);
    } else {
      computeXYVelocity(dx, dy, vx, vy);
      vz        = (dist_xy < landing_radius_) ? -std::fabs(z_descent_) : 0.0;
      yaw_speed = std::clamp(yaw_kp_ * yaw_err, -yaw_speed_max_, yaw_speed_max_);
    }

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[p] vx=%.3f vy=%.3f vz=%.3f yaw_sp=%.3f | dist_xy=%.3f dz=%.3f yaw_err=%.3f",
                vx, vy, vz, yaw_speed, dist_xy, dz, yaw_err);

    if (!speed_motion_handler_->sendSpeedCommandWithYawSpeed("earth", vx, vy, vz, yaw_speed)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "[p] Error sending speed command");
      result_.precision_landing_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    feedback_.distance_xy = dist_xy;
    feedback_.distance_z  = dz;
    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  enum class AlignState { ALIGNING_XY, ALIGNING_YAW, DESCENDING };

  std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> speed_motion_handler_{nullptr};

  std::string marker_frame_id_{"landing_pad"};

  // Plugin-specific
  double xy_kp_{1.0};
  double yaw_kp_{1.0};

  // Shared
  double z_descent_{0.3};
  double z_distance_threshold_{0.1};
  double xy_speed_max_{1.0};
  double landing_radius_{0.3};
  double yaw_speed_max_{0.5};

  bool   yaw_detached_{false};
  double yaw_detached_xy_threshold_{0.3};
  double yaw_detached_yaw_threshold_{0.1};

  AlignState   state_{AlignState::ALIGNING_XY};
  rclcpp::Time last_aruco_time_;

  void resetStatus()
  {
    state_           = AlignState::ALIGNING_XY;
    last_aruco_time_ = node_ptr_->now();
    RCLCPP_INFO(node_ptr_->get_logger(), "[p] Status reset");
  }

  void computeXYVelocity(double dx, double dy, double & vx, double & vy)
  {
    vx = xy_kp_ * dx;
    vy = xy_kp_ * dy;
    const double vxy = std::hypot(vx, vy);
    if (vxy > xy_speed_max_) {
      const double s = xy_speed_max_ / (vxy + 1e-9);
      vx *= s;
      vy *= s;
    }
  }

  void runAlignmentFSM(double dx, double dy, double dist_xy, double yaw_err,
                       double & vx, double & vy, double & vz, double & yaw_speed)
  {
    switch (state_) {
      case AlignState::ALIGNING_XY:
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
                             "[p] ALIGNING_XY  dist_xy=%.3f (th=%.3f)",
                             dist_xy, yaw_detached_xy_threshold_);
        computeXYVelocity(dx, dy, vx, vy);
        vz = 0.0;
        yaw_speed = 0.0;
        if (dist_xy < yaw_detached_xy_threshold_) {
          RCLCPP_INFO(node_ptr_->get_logger(), "[p] XY aligned -> ALIGNING_YAW");
          state_ = AlignState::ALIGNING_YAW;
        }
        break;

      case AlignState::ALIGNING_YAW:
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
                             "[p] ALIGNING_YAW  yaw_err=%.3f (th=%.3f)",
                             yaw_err, yaw_detached_yaw_threshold_);
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
        yaw_speed = std::clamp(yaw_kp_ * yaw_err, -yaw_speed_max_, yaw_speed_max_);
        if (std::fabs(yaw_err) < yaw_detached_yaw_threshold_) {
          RCLCPP_INFO(node_ptr_->get_logger(), "[p] YAW aligned -> DESCENDING");
          state_ = AlignState::DESCENDING;
        }
        break;

      default:
        break;
    }
  }

  bool tryGetArucoTF(geometry_msgs::msg::TransformStamped & tf_out)
  {
    try {
      tf_out           = tf_handler_->getTransform("earth", marker_frame_id_);
      last_aruco_time_ = node_ptr_->now();
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), "[p] TF unavailable: %s", ex.what());
      return false;
    }
  }

  bool arucoTimeoutExceeded() const
  {
    return (node_ptr_->now() - last_aruco_time_).seconds() > params_.aruco_timeout_threshold;
  }

  std::tuple<double, double, double> computeRelativeError(
      const geometry_msgs::msg::TransformStamped & tf_aruco)
  {
    const auto & p = actual_pose_.pose.position;
    const auto & t = tf_aruco.transform.translation;
    return {t.x - p.x, t.y - p.y, t.z - p.z};
  }
};

}  // namespace precision_landing_plugin_p

PLUGINLIB_EXPORT_CLASS(precision_landing_plugin_p::Plugin,
                       precision_landing_base::PrecisionLandingBase)

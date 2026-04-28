// BSD-3-Clause
/**
 * @file precision_landing_plugin_pid.cpp
 *
 * Plugin: full PID for XY + proportional yaw matching + constant descent inside radius.
 *
 * XY position error is controlled with independent P+I+D loops that handle
 * steady-state disturbances (wind, sensor bias) via the integral term and
 * damp oscillations via the derivative term.
 *
 * Yaw is matched to the landing pad orientation extracted from TF using a
 * proportional controller (yaw_speed = kp * yaw_error).
 *
 * Descent is enabled only when the XY error is below pid_landing_radius,
 * ensuring the drone is centred before committing to the approach.
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

namespace precision_landing_plugin_pid
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
    RCLCPP_INFO(node_ptr_->get_logger(), "[pid] Init start");

    speed_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(node_ptr_);

    node_ptr_->declare_parameter<std::string>("marker_frame_id", "landing_pad");
    node_ptr_->get_parameter("marker_frame_id", marker_frame_id_);

    // XY PID gains
    node_ptr_->declare_parameter<double>("pid_xy_kp", 1.0);
    node_ptr_->get_parameter("pid_xy_kp", pid_xy_kp_);
    node_ptr_->declare_parameter<double>("pid_xy_ki", 0.05);
    node_ptr_->get_parameter("pid_xy_ki", pid_xy_ki_);
    node_ptr_->declare_parameter<double>("pid_xy_kd", 0.15);
    node_ptr_->get_parameter("pid_xy_kd", pid_xy_kd_);
    node_ptr_->declare_parameter<double>("pid_xy_speed_max", 1.0);
    node_ptr_->get_parameter("pid_xy_speed_max", pid_xy_speed_max_);
    node_ptr_->declare_parameter<double>("pid_xy_integral_max", 0.3);
    node_ptr_->get_parameter("pid_xy_integral_max", pid_xy_integral_max_);

    // Z descent
    node_ptr_->declare_parameter<double>("pid_z_descent", 0.3);
    node_ptr_->get_parameter("pid_z_descent", pid_z_descent_);
    node_ptr_->declare_parameter<double>("pid_z_distance_threshold", 0.1);
    node_ptr_->get_parameter("pid_z_distance_threshold", pid_z_distance_threshold_);
    node_ptr_->declare_parameter<double>("pid_landing_radius", 0.3);
    node_ptr_->get_parameter("pid_landing_radius", pid_landing_radius_);

    // Yaw matching
    node_ptr_->declare_parameter<double>("pid_yaw_kp", 1.0);
    node_ptr_->get_parameter("pid_yaw_kp", pid_yaw_kp_);
    node_ptr_->declare_parameter<double>("pid_yaw_speed_max", 0.5);
    node_ptr_->get_parameter("pid_yaw_speed_max", pid_yaw_speed_max_);

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "[pid] xy_kp=%.2f xy_ki=%.2f xy_kd=%.2f xy_vmax=%.2f xy_imax=%.2f"
        " | z_descent=%.2f z_th=%.2f radius=%.2f"
        " | yaw_kp=%.2f yaw_vmax=%.2f | marker=%s",
        pid_xy_kp_, pid_xy_ki_, pid_xy_kd_, pid_xy_speed_max_, pid_xy_integral_max_,
        pid_z_descent_, pid_z_distance_threshold_, pid_landing_radius_,
        pid_yaw_kp_, pid_yaw_speed_max_, marker_frame_id_.c_str());

    resetStatus();
  }

  bool own_activate(as2_msgs::action::PrecisionLanding::Goal &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[pid] Precision Landing accepted");
    resetStatus();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[pid] Precision Landing canceled, hover");
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[pid] Precision Landing paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[pid] Precision Landing resumed");
    resetStatus();
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[pid] end - state: %d", (int)state);
    if (state != as2_behavior::ExecutionStatus::SUCCESS)
      sendHover();
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    geometry_msgs::msg::TransformStamped tf_aruco;
    if (!tryGetArucoTF(tf_aruco)) {
      if (arucoTimeoutExceeded()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "[pid] ArUco TF timeout -> failure");
        result_.precision_landing_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
      RCLCPP_INFO(node_ptr_->get_logger(), "[pid] No ArUco TF yet, hovering...");
      resetPIDState();
      sendHover();
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    // Compute dt for I and D terms; clamp to avoid spikes after pauses.
    const rclcpp::Time now = node_ptr_->now();
    const double dt        = last_run_valid_ ? (now - last_run_time_).seconds() : 0.0;
    last_run_time_         = now;
    last_run_valid_        = true;
    const double dt_c      = std::min(dt, 0.1);

    const auto [dx, dy, dz] = computeRelativeError(tf_aruco);
    const double dist_xy    = std::hypot(dx, dy);
    const double abs_dz     = std::fabs(dz);

    if (abs_dz < pid_z_distance_threshold_) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[pid] z threshold reached (|dz|=%.3f). Success.", abs_dz);
      result_.precision_landing_success = true;
      sendHover();
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    // --- XY PID ---
    if (dt_c > 1e-6) {
      x_integral_ = std::clamp(
          x_integral_ + dx * dt_c, -pid_xy_integral_max_, pid_xy_integral_max_);
      y_integral_ = std::clamp(
          y_integral_ + dy * dt_c, -pid_xy_integral_max_, pid_xy_integral_max_);
    }
    const double dx_d = (dt_c > 1e-6) ? (dx - x_prev_err_) / dt_c : 0.0;
    const double dy_d = (dt_c > 1e-6) ? (dy - y_prev_err_) / dt_c : 0.0;
    x_prev_err_       = dx;
    y_prev_err_       = dy;

    double vx = pid_xy_kp_ * dx + pid_xy_ki_ * x_integral_ + pid_xy_kd_ * dx_d;
    double vy = pid_xy_kp_ * dy + pid_xy_ki_ * y_integral_ + pid_xy_kd_ * dy_d;

    const double vxy = std::hypot(vx, vy);
    if (vxy > pid_xy_speed_max_) {
      const double s = pid_xy_speed_max_ / (vxy + 1e-9);
      vx *= s;
      vy *= s;
    }

    // --- Z: constant descent only when centred ---
    const double vz = (dist_xy < pid_landing_radius_) ? -std::fabs(pid_z_descent_) : 0.0;

    // --- Yaw: P controller on pad orientation from TF ---
    const double pad_yaw =
        static_cast<double>(as2::frame::getYawFromQuaternion(tf_aruco.transform.rotation));
    const double drone_yaw = static_cast<double>(getActualYaw());
    const double yaw_err   = normalizeAngle(pad_yaw - drone_yaw);
    const double yaw_speed =
        std::clamp(pid_yaw_kp_ * yaw_err, -pid_yaw_speed_max_, pid_yaw_speed_max_);

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[pid] vx=%.3f vy=%.3f vz=%.3f yaw_sp=%.3f | "
                "dist_xy=%.3f dz=%.3f yaw_err=%.3f dt=%.3f",
                vx, vy, vz, yaw_speed, dist_xy, dz, yaw_err, dt_c);

    if (!speed_motion_handler_->sendSpeedCommandWithYawSpeed(
            "earth", vx, vy, vz, yaw_speed)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "[pid] Error sending speed command");
      result_.precision_landing_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    feedback_.distance_xy = dist_xy;
    feedback_.distance_z  = dz;

    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> speed_motion_handler_{nullptr};

  std::string marker_frame_id_{"landing_pad"};

  double pid_xy_kp_{1.0};
  double pid_xy_ki_{0.05};
  double pid_xy_kd_{0.15};
  double pid_xy_speed_max_{1.0};
  double pid_xy_integral_max_{0.3};
  double pid_z_descent_{0.3};
  double pid_z_distance_threshold_{0.1};
  double pid_landing_radius_{0.3};
  double pid_yaw_kp_{1.0};
  double pid_yaw_speed_max_{0.5};

  // PID state
  double x_integral_{0.0};
  double y_integral_{0.0};
  double x_prev_err_{0.0};
  double y_prev_err_{0.0};
  rclcpp::Time last_run_time_;
  bool last_run_valid_{false};

  bool last_aruco_ok_{false};
  rclcpp::Time last_aruco_time_;

  void resetStatus()
  {
    last_aruco_ok_   = false;
    last_aruco_time_ = node_ptr_->now();
    resetPIDState();
    RCLCPP_INFO(node_ptr_->get_logger(), "[pid] Status reset");
  }

  void resetPIDState()
  {
    x_integral_    = 0.0;
    y_integral_    = 0.0;
    x_prev_err_    = 0.0;
    y_prev_err_    = 0.0;
    last_run_valid_ = false;
  }

  bool tryGetArucoTF(geometry_msgs::msg::TransformStamped & tf_out)
  {
    try {
      tf_out         = tf_handler_->getTransform("earth", marker_frame_id_);
      last_aruco_ok_ = true;
      last_aruco_time_ = node_ptr_->now();
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), "[pid] TF unavailable: %s", ex.what());
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

}  // namespace precision_landing_plugin_pid

PLUGINLIB_EXPORT_CLASS(precision_landing_plugin_pid::Plugin,
                       precision_landing_base::PrecisionLandingBase)

// BSD-3-Clause
/**
 * @file precision_landing_plugin_align_land.cpp
 *
 * Plugin: two-phase state machine — ALIGNING then DESCENDING.
 *
 * ALIGNING phase:
 *   The drone corrects XY position and yaw to match the landing pad. No
 *   altitude change is commanded. The drone transitions to DESCENDING only
 *   when both dist_xy < align_xy_threshold AND |yaw_error| < align_yaw_threshold.
 *
 * DESCENDING phase:
 *   The drone descends at a constant speed while continuing to correct XY and
 *   yaw. If dist_xy > realign_xy_threshold OR |yaw_error| > realign_yaw_threshold
 *   (hysteresis: realign > align thresholds), it transitions back to ALIGNING
 *   so the approach is re-established before continuing the descent.
 *
 * This guarantees alignment before each descent increment, making it suitable
 * for platforms with strict touchdown requirements.
 */

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"
#include "precision_landing_behavior/precision_landing_base.hpp"

namespace precision_landing_plugin_align_land
{

static double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

enum class State { ALIGNING, DESCENDING };

class Plugin : public precision_landing_base::PrecisionLandingBase
{
public:
  void ownInit() override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[align_land] Init start");

    speed_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(node_ptr_);

    node_ptr_->declare_parameter<std::string>("marker_frame_id", "landing_pad");
    node_ptr_->get_parameter("marker_frame_id", marker_frame_id_);

    // Thresholds to enter DESCENDING (must be tight)
    node_ptr_->declare_parameter<double>("al_align_xy_threshold", 0.15);
    node_ptr_->get_parameter("al_align_xy_threshold", al_align_xy_threshold_);
    node_ptr_->declare_parameter<double>("al_align_yaw_threshold", 0.1);
    node_ptr_->get_parameter("al_align_yaw_threshold", al_align_yaw_threshold_);

    // Thresholds to return to ALIGNING (hysteresis: must be > align thresholds)
    node_ptr_->declare_parameter<double>("al_realign_xy_threshold", 0.3);
    node_ptr_->get_parameter("al_realign_xy_threshold", al_realign_xy_threshold_);
    node_ptr_->declare_parameter<double>("al_realign_yaw_threshold", 0.2);
    node_ptr_->get_parameter("al_realign_yaw_threshold", al_realign_yaw_threshold_);

    // Descent
    node_ptr_->declare_parameter<double>("al_descent_speed", 0.3);
    node_ptr_->get_parameter("al_descent_speed", al_descent_speed_);
    node_ptr_->declare_parameter<double>("al_z_distance_threshold", 0.1);
    node_ptr_->get_parameter("al_z_distance_threshold", al_z_distance_threshold_);

    // XY correction (P controller)
    node_ptr_->declare_parameter<double>("al_xy_gain", 1.0);
    node_ptr_->get_parameter("al_xy_gain", al_xy_gain_);
    node_ptr_->declare_parameter<double>("al_xy_speed_max", 1.0);
    node_ptr_->get_parameter("al_xy_speed_max", al_xy_speed_max_);

    // Yaw correction (P controller)
    node_ptr_->declare_parameter<double>("al_yaw_kp", 1.2);
    node_ptr_->get_parameter("al_yaw_kp", al_yaw_kp_);
    node_ptr_->declare_parameter<double>("al_yaw_speed_max", 0.6);
    node_ptr_->get_parameter("al_yaw_speed_max", al_yaw_speed_max_);

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "[align_land] align_xy=%.3f align_yaw=%.3f | realign_xy=%.3f realign_yaw=%.3f"
        " | descent=%.2f z_th=%.2f | xy_gain=%.2f xy_vmax=%.2f"
        " | yaw_kp=%.2f yaw_vmax=%.2f | marker=%s",
        al_align_xy_threshold_, al_align_yaw_threshold_,
        al_realign_xy_threshold_, al_realign_yaw_threshold_,
        al_descent_speed_, al_z_distance_threshold_,
        al_xy_gain_, al_xy_speed_max_,
        al_yaw_kp_, al_yaw_speed_max_, marker_frame_id_.c_str());

    resetStatus();
  }

  bool own_activate(as2_msgs::action::PrecisionLanding::Goal &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[align_land] Precision Landing accepted");
    resetStatus();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[align_land] Precision Landing canceled, hover");
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[align_land] Precision Landing paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[align_land] Precision Landing resumed");
    resetStatus();
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[align_land] end - state: %d", (int)state);
    if (state != as2_behavior::ExecutionStatus::SUCCESS)
      sendHover();
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    geometry_msgs::msg::TransformStamped tf_aruco;
    if (!tryGetArucoTF(tf_aruco)) {
      if (arucoTimeoutExceeded()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "[align_land] ArUco TF timeout -> failure");
        result_.precision_landing_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
      RCLCPP_INFO(node_ptr_->get_logger(), "[align_land] No ArUco TF yet, hovering...");
      state_ = State::ALIGNING;
      sendHover();
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    const auto [dx, dy, dz] = computeRelativeError(tf_aruco);
    const double dist_xy    = std::hypot(dx, dy);
    const double abs_dz     = std::fabs(dz);

    // Yaw error: pad orientation minus drone orientation, wrapped to [-π, π]
    const double pad_yaw   =
        static_cast<double>(as2::frame::getYawFromQuaternion(tf_aruco.transform.rotation));
    const double drone_yaw = static_cast<double>(getActualYaw());
    const double yaw_err   = normalizeAngle(pad_yaw - drone_yaw);
    const double abs_yaw   = std::fabs(yaw_err);

    // Success
    if (abs_dz < al_z_distance_threshold_) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[align_land] z threshold reached (|dz|=%.3f). Success.", abs_dz);
      result_.precision_landing_success = true;
      sendHover();
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    // --- State transitions with hysteresis ---
    if (state_ == State::ALIGNING) {
      if (dist_xy < al_align_xy_threshold_ && abs_yaw < al_align_yaw_threshold_) {
        state_ = State::DESCENDING;
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[align_land] ALIGNING -> DESCENDING (dist_xy=%.3f yaw_err=%.3f)",
                    dist_xy, yaw_err);
      }
    } else {  // DESCENDING
      if (dist_xy > al_realign_xy_threshold_ || abs_yaw > al_realign_yaw_threshold_) {
        state_ = State::ALIGNING;
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "[align_land] DESCENDING -> ALIGNING (dist_xy=%.3f yaw_err=%.3f)",
                    dist_xy, yaw_err);
      }
    }

    // --- XY correction (P controller, always active) ---
    double vx = al_xy_gain_ * dx;
    double vy = al_xy_gain_ * dy;
    const double vxy = std::hypot(vx, vy);
    if (vxy > al_xy_speed_max_) {
      const double s = al_xy_speed_max_ / (vxy + 1e-9);
      vx *= s;
      vy *= s;
    }

    // --- Yaw correction (P controller, always active) ---
    const double yaw_speed =
        std::clamp(al_yaw_kp_ * yaw_err, -al_yaw_speed_max_, al_yaw_speed_max_);

    // --- Z: descend only in DESCENDING phase ---
    const double vz = (state_ == State::DESCENDING) ? -std::fabs(al_descent_speed_) : 0.0;

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[align_land] %s | vx=%.3f vy=%.3f vz=%.3f yaw_sp=%.3f | "
                "dist_xy=%.3f dz=%.3f yaw_err=%.3f",
                (state_ == State::ALIGNING) ? "ALIGNING" : "DESCENDING",
                vx, vy, vz, yaw_speed, dist_xy, dz, yaw_err);

    if (!speed_motion_handler_->sendSpeedCommandWithYawSpeed(
            "earth", vx, vy, vz, yaw_speed)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "[align_land] Error sending speed command");
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

  double al_align_xy_threshold_{0.15};
  double al_align_yaw_threshold_{0.1};
  double al_realign_xy_threshold_{0.3};
  double al_realign_yaw_threshold_{0.2};
  double al_descent_speed_{0.3};
  double al_z_distance_threshold_{0.1};
  double al_xy_gain_{1.0};
  double al_xy_speed_max_{1.0};
  double al_yaw_kp_{1.2};
  double al_yaw_speed_max_{0.6};

  State state_{State::ALIGNING};

  bool last_aruco_ok_{false};
  rclcpp::Time last_aruco_time_;

  void resetStatus()
  {
    last_aruco_ok_   = false;
    last_aruco_time_ = node_ptr_->now();
    state_           = State::ALIGNING;
    RCLCPP_INFO(node_ptr_->get_logger(), "[align_land] Status reset -> ALIGNING");
  }

  bool tryGetArucoTF(geometry_msgs::msg::TransformStamped & tf_out)
  {
    try {
      tf_out         = tf_handler_->getTransform("earth", marker_frame_id_);
      last_aruco_ok_ = true;
      last_aruco_time_ = node_ptr_->now();
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), "[align_land] TF unavailable: %s", ex.what());
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

}  // namespace precision_landing_plugin_align_land

PLUGINLIB_EXPORT_CLASS(precision_landing_plugin_align_land::Plugin,
                       precision_landing_base::PrecisionLandingBase)

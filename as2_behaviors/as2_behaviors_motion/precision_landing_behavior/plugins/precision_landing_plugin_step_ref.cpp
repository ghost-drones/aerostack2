// BSD-3-Clause
/**
 * @file precision_landing_plugin_step_ref.cpp
 *
 * Plugin: staircase (step-reference) descent.
 * Inside the XY cone the drone tracks a reference that is decremented by
 * step_ref_descent_step each time it is reached, producing a staircase
 * descent.  Outside the cone XY position is corrected and altitude is
 * held at a safe reference height.
 *
 * Adapts the "placeholder" Zcontrol strategy from ARIEL landing.cpp.
 */

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"
#include "precision_landing_behavior/precision_landing_base.hpp"

namespace precision_landing_plugin_step_ref
{

class Plugin : public precision_landing_base::PrecisionLandingBase
{
public:
  void ownInit() override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[step_ref] Init start");

    speed_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(node_ptr_);

    node_ptr_->declare_parameter<std::string>("marker_frame_id", "landing_pad");
    node_ptr_->get_parameter("marker_frame_id", marker_frame_id_);

    node_ptr_->declare_parameter<double>("step_ref_descent_step", 0.3);
    node_ptr_->get_parameter("step_ref_descent_step", step_ref_descent_step_);

    node_ptr_->declare_parameter<double>("step_ref_z_gain", 1.0);
    node_ptr_->get_parameter("step_ref_z_gain", step_ref_z_gain_);

    node_ptr_->declare_parameter<double>("step_ref_speed_up", 0.2);
    node_ptr_->get_parameter("step_ref_speed_up", step_ref_speed_up_);

    node_ptr_->declare_parameter<double>("step_ref_z_distance_threshold", 0.1);
    node_ptr_->get_parameter("step_ref_z_distance_threshold", step_ref_z_distance_threshold_);

    node_ptr_->declare_parameter<double>("step_ref_safe_ref_altitude", 3.5);
    node_ptr_->get_parameter("step_ref_safe_ref_altitude", step_ref_safe_ref_altitude_);

    node_ptr_->declare_parameter<double>("step_ref_safe_ascent_altitude", 3.0);
    node_ptr_->get_parameter("step_ref_safe_ascent_altitude", step_ref_safe_ascent_altitude_);

    node_ptr_->declare_parameter<double>("step_ref_xy_gain", 1.0);
    node_ptr_->get_parameter("step_ref_xy_gain", step_ref_xy_gain_);

    node_ptr_->declare_parameter<double>("step_ref_xy_speed_max", 1.0);
    node_ptr_->get_parameter("step_ref_xy_speed_max", step_ref_xy_speed_max_);

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "[step_ref] Params -> step: %.2f | z_gain: %.2f | speed_up: %.2f | z_th: %.2f"
        " | safe_ref: %.2f | safe_asc: %.2f | xy_gain: %.2f | xy_vmax: %.2f | marker: %s",
        step_ref_descent_step_, step_ref_z_gain_, step_ref_speed_up_,
        step_ref_z_distance_threshold_, step_ref_safe_ref_altitude_,
        step_ref_safe_ascent_altitude_, step_ref_xy_gain_,
        step_ref_xy_speed_max_, marker_frame_id_.c_str());

    resetStatus();
  }

  bool own_activate(as2_msgs::action::PrecisionLanding::Goal &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[step_ref] Precision Landing accepted");
    resetStatus();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[step_ref] Precision Landing canceled, hover");
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[step_ref] Precision Landing paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[step_ref] Precision Landing resumed");
    resetStatus();
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[step_ref] end - state: %d", (int)state);
    if (state != as2_behavior::ExecutionStatus::SUCCESS)
      sendHover();
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    geometry_msgs::msg::TransformStamped tf_aruco;
    if (!tryGetArucoTF(tf_aruco)) {
      if (arucoTimeoutExceeded()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "[step_ref] ArUco TF timeout -> failure");
        result_.precision_landing_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
      RCLCPP_INFO(node_ptr_->get_logger(), "[step_ref] No ArUco TF yet, hovering...");
      sendHover();
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    const auto [dx, dy, dz] = computeRelativeError(tf_aruco);
    const double dist_xy    = std::hypot(dx, dy);
    const double abs_dz     = std::fabs(dz);

    if (abs_dz < step_ref_z_distance_threshold_) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[step_ref] Reached z threshold (|dz|=%.2f). Success.", abs_dz);
      result_.precision_landing_success = true;
      sendHover();
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    // Cone radius scales with altitude: 1 m at ≥ 2.5 m, tighter below that.
    const double cone_radius = (abs_dz > 2.5) ? 1.0 : (abs_dz / 2.5);

    double vx = 0.0, vy = 0.0, vz = 0.0;

    if (dist_xy <= cone_radius) {
      // --- Inside cone: step-reference descent ---
      if (first_step_) {
        z_ref_      = abs_dz - step_ref_descent_step_;
        first_step_ = false;
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[step_ref] First step initialised -> z_ref=%.2f", z_ref_);
      }

      // Step the reference down once the current altitude reaches it.
      if (abs_dz <= z_ref_) {
        z_ref_ -= step_ref_descent_step_;
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[step_ref] Reference stepped down -> z_ref=%.2f", z_ref_);
      }

      // Proportional command toward the current step reference (negative = descend).
      vz = (z_ref_ - abs_dz) * step_ref_z_gain_;

      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[step_ref] Inside cone | vz=%.3f | z_ref=%.2f | abs_dz=%.2f",
                  vz, z_ref_, abs_dz);
    } else {
      // --- Outside cone: correct XY, hold / recover altitude ---
      vx = step_ref_xy_gain_ * dx;
      vy = step_ref_xy_gain_ * dy;
      const double vxy = std::hypot(vx, vy);
      if (vxy > step_ref_xy_speed_max_) {
        const double s = step_ref_xy_speed_max_ / (vxy + 1e-6);
        vx *= s;
        vy *= s;
      }

      // Reset so the next cone entry starts with a fresh reference.
      first_step_ = true;

      if (abs_dz < step_ref_safe_ascent_altitude_) {
        vz = step_ref_speed_up_;   // ascend for safety when too close to pad
      } else {
        // Proportional altitude hold at safe reference height.
        vz = (step_ref_safe_ref_altitude_ - abs_dz) * step_ref_z_gain_;
      }

      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[step_ref] Outside cone | correcting XY | vz=%.3f", vz);
    }

    if (!speed_motion_handler_->sendSpeedCommandWithYawSpeed("earth", vx, vy, vz, 0.0)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "[step_ref] Error sending speed command");
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
  double step_ref_descent_step_{0.3};
  double step_ref_z_gain_{1.0};
  double step_ref_speed_up_{0.2};
  double step_ref_z_distance_threshold_{0.1};
  double step_ref_safe_ref_altitude_{3.5};
  double step_ref_safe_ascent_altitude_{3.0};
  double step_ref_xy_gain_{1.0};
  double step_ref_xy_speed_max_{1.0};

  bool last_aruco_ok_{false};
  rclcpp::Time last_aruco_time_;

  double z_ref_{0.0};
  bool   first_step_{true};

  void resetStatus()
  {
    last_aruco_ok_  = false;
    last_aruco_time_ = node_ptr_->now();
    first_step_ = true;
    z_ref_      = 0.0;
    RCLCPP_INFO(node_ptr_->get_logger(), "[step_ref] Status reset");
  }

  bool tryGetArucoTF(geometry_msgs::msg::TransformStamped & tf_out)
  {
    try {
      tf_out = tf_handler_->getTransform("earth", marker_frame_id_);
      last_aruco_ok_   = true;
      last_aruco_time_ = node_ptr_->now();
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), "[step_ref] TF unavailable: %s", ex.what());
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

}  // namespace precision_landing_plugin_step_ref

PLUGINLIB_EXPORT_CLASS(precision_landing_plugin_step_ref::Plugin,
                       precision_landing_base::PrecisionLandingBase)

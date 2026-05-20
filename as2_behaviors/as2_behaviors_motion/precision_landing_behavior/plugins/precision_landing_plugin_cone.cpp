// BSD-3-Clause
/**
 * @file precision_landing_plugin_cone.cpp
 *
 * Plugin: cone-based adaptive descent velocity.
 *
 * Descent speed is computed from three terms:
 *   - cone : penalises descent when the drone is laterally off-centre and
 *             close to the pad (ratio dist_xy / (cone_gain * altitude)).
 *   - aversion : additional penalty proportional to dist_xy / altitude^2,
 *                discouraging descent when off-centre near the ground.
 *   - brake : linear braking term that limits descent near the landing gear
 *             height to prevent hard touchdown.
 *
 * XY position is corrected in parallel with a simple P-controller.
 *
 * Adapts the "reference" Zcontrol strategy from ARIEL landing.cpp.
 */

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"
#include "precision_landing_behavior/precision_landing_base.hpp"

namespace precision_landing_plugin_cone
{

class Plugin : public precision_landing_base::PrecisionLandingBase
{
public:
  void ownInit() override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[cone] Init start");

    speed_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(node_ptr_);

    node_ptr_->declare_parameter<std::string>("marker_frame_id", "landing_pad");
    node_ptr_->get_parameter("marker_frame_id", marker_frame_id_);

    // Shared parameters
    node_ptr_->declare_parameter<double>("z_distance_threshold", 0.1);
    node_ptr_->get_parameter("z_distance_threshold", cone_z_distance_threshold_);

    // Maximum downward speed (m/s, used as saturation and as the free-descent
    // value above cone_max_height).
    node_ptr_->declare_parameter<double>("z_descent", 0.3);
    node_ptr_->get_parameter("z_descent", cone_down_sat_);

    // Inverse gain: lower value → more aggressive descent when centred.
    node_ptr_->declare_parameter<double>("cone_inv_gain", 1.0);
    node_ptr_->get_parameter("cone_inv_gain", cone_inv_gain_);

    // Higher cone_gain → wider cone (permits descent from further off-centre).
    node_ptr_->declare_parameter<double>("cone_gain", 3.0);
    node_ptr_->get_parameter("cone_gain", cone_gain_);

    node_ptr_->declare_parameter<double>("cone_offset", 0.1);
    node_ptr_->get_parameter("cone_offset", cone_offset_);

    // Above this altitude the cone constraint is relaxed and full descent is allowed.
    node_ptr_->declare_parameter<double>("cone_max_height", 2.0);
    node_ptr_->get_parameter("cone_max_height", cone_max_height_);

    node_ptr_->declare_parameter<double>("cone_aversion_gain", 0.5);
    node_ptr_->get_parameter("cone_aversion_gain", cone_aversion_gain_);

    // Braking: cone_brake_gain * (altitude - (cone_gear_offset - cone_brake_alti))
    node_ptr_->declare_parameter<double>("cone_brake_gain", 1.0);
    node_ptr_->get_parameter("cone_brake_gain", cone_brake_gain_);
    node_ptr_->declare_parameter<double>("cone_gear_offset", 0.45);
    node_ptr_->get_parameter("cone_gear_offset", cone_gear_offset_);
    node_ptr_->declare_parameter<double>("cone_brake_alti", 0.3);
    node_ptr_->get_parameter("cone_brake_alti", cone_brake_alti_);

    // Below this altitude the drone holds position instead of descending.
    node_ptr_->declare_parameter<double>("cone_min_height", 0.1);
    node_ptr_->get_parameter("cone_min_height", cone_min_height_);

    node_ptr_->declare_parameter<double>("cone_xy_gain", 1.0);
    node_ptr_->get_parameter("cone_xy_gain", cone_xy_gain_);
    node_ptr_->declare_parameter<double>("xy_speed_max", 1.0);
    node_ptr_->get_parameter("xy_speed_max", cone_xy_speed_max_);

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "[cone] Params -> down_sat: %.2f | inv_gain: %.2f | cone_gain: %.2f"
        " | cone_offset: %.2f | max_h: %.2f | aversion: %.2f"
        " | brake_gain: %.2f | gear_off: %.2f | brake_alti: %.2f"
        " | min_h: %.2f | xy_gain: %.2f | xy_vmax: %.2f | marker: %s",
        cone_down_sat_, cone_inv_gain_, cone_gain_, cone_offset_,
        cone_max_height_, cone_aversion_gain_,
        cone_brake_gain_, cone_gear_offset_, cone_brake_alti_,
        cone_min_height_, cone_xy_gain_, cone_xy_speed_max_,
        marker_frame_id_.c_str());

    resetStatus();
  }

  bool own_activate(as2_msgs::action::PrecisionLanding::Goal &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[cone] Precision Landing accepted");
    resetStatus();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[cone] Precision Landing canceled, hover");
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[cone] Precision Landing paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> &) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[cone] Precision Landing resumed");
    resetStatus();
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[cone] end - state: %d", (int)state);
    if (state != as2_behavior::ExecutionStatus::SUCCESS)
      sendHover();
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    geometry_msgs::msg::TransformStamped tf_aruco;
    if (!tryGetArucoTF(tf_aruco)) {
      if (arucoTimeoutExceeded()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "[cone] ArUco TF timeout -> failure");
        result_.precision_landing_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
      RCLCPP_INFO(node_ptr_->get_logger(), "[cone] No ArUco TF yet, hovering...");
      sendHover();
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    const auto [dx, dy, dz] = computeRelativeError(tf_aruco);
    const double dist_xy    = std::hypot(dx, dy);
    const double abs_dz     = std::fabs(dz);

    if (abs_dz < cone_z_distance_threshold_) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[cone] Reached z threshold (|dz|=%.2f). Success.", abs_dz);
      result_.precision_landing_success = true;
      sendHover();
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    const double vz         = computeDescentVelocity(dist_xy, abs_dz);
    const auto [vx, vy]     = computeXYVelocity(dx, dy, dist_xy);

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[cone] cmd vx=%.3f vy=%.3f vz=%.3f | dist_xy=%.3f abs_dz=%.3f",
                vx, vy, vz, dist_xy, abs_dz);

    if (!speed_motion_handler_->sendSpeedCommandWithYawSpeed("earth", vx, vy, vz, 0.0)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "[cone] Error sending speed command");
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
  double cone_z_distance_threshold_{0.1};
  double cone_down_sat_{0.3};
  double cone_inv_gain_{1.0};
  double cone_gain_{3.0};
  double cone_offset_{0.1};
  double cone_max_height_{2.0};
  double cone_aversion_gain_{0.5};
  double cone_brake_gain_{1.0};
  double cone_gear_offset_{0.45};
  double cone_brake_alti_{0.3};
  double cone_min_height_{0.1};
  double cone_xy_gain_{1.0};
  double cone_xy_speed_max_{1.0};

  bool last_aruco_ok_{false};
  rclcpp::Time last_aruco_time_;

  void resetStatus()
  {
    last_aruco_ok_   = false;
    last_aruco_time_ = node_ptr_->now();
    RCLCPP_INFO(node_ptr_->get_logger(), "[cone] Status reset");
  }

  bool tryGetArucoTF(geometry_msgs::msg::TransformStamped & tf_out)
  {
    try {
      tf_out = tf_handler_->getTransform("earth", marker_frame_id_);
      last_aruco_ok_   = true;
      last_aruco_time_ = node_ptr_->now();
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), "[cone] TF unavailable: %s", ex.what());
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

  // Cone-based adaptive descent velocity (negative = down in ENU).
  double computeDescentVelocity(double dist, double altitude) const
  {
    double cone     = 0.0;
    double aversion = 0.0;

    if (altitude < std::fabs(cone_max_height_)) {
      // Cone value: large when far from centre at low altitude.
      cone     = std::min(dist / (cone_gain_ * altitude) - cone_offset_, 0.4);
      // Aversion: penalises descent proportional to dist / altitude^2.
      aversion = std::min(cone_aversion_gain_ * dist / (altitude * altitude), 1.0);
    } else {
      // Above max_height the full saturation speed is allowed.
      cone     = -std::fabs(cone_down_sat_);
      aversion = 0.0;
    }

    double pos_gain;
    if (dist >= 1e-4) {
      const double brake_limit =
          cone_brake_gain_ * (altitude - (cone_gear_offset_ - cone_brake_alti_));
      pos_gain = std::min(
          1.0 / (cone_inv_gain_ * dist) - cone - aversion,
          std::min(brake_limit, std::fabs(cone_down_sat_)));
    } else {
      pos_gain = 0.3;  // default small descent when nearly centred
    }

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[cone] cone=%.3f aversion=%.3f pos_gain=%.3f", cone, aversion, pos_gain);

    if (altitude >= std::fabs(cone_min_height_)) {
      return -pos_gain;
    }
    // Very close to the ground: hold altitude to avoid hard contact.
    return 0.05;
  }

  std::pair<double, double> computeXYVelocity(double dx, double dy, double dist_xy) const
  {
    double vx = cone_xy_gain_ * dx;
    double vy = cone_xy_gain_ * dy;
    const double vxy = std::hypot(vx, vy);
    if (vxy > cone_xy_speed_max_) {
      const double s = cone_xy_speed_max_ / (vxy + 1e-6);
      vx *= s;
      vy *= s;
    }
    return {vx, vy};
  }
};

}  // namespace precision_landing_plugin_cone

PLUGINLIB_EXPORT_CLASS(precision_landing_plugin_cone::Plugin,
                       precision_landing_base::PrecisionLandingBase)

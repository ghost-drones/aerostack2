// BSD-3-Clause
/**
 * @file precision_landing_plugin_vconstant.cpp
 *
 * Plugin: dentro do raio XY, desce a v constante; fora do raio, corrige XY.
 * Usa somente SpeedMotion.
 * OK Lucca 08/10 (rev: integração com tf_handler_ igual ao LandBehavior)
 */

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"
#include "precision_landing_behavior/precision_landing_base.hpp"

namespace precision_landing_plugin_vconstant
{

class Plugin : public precision_landing_base::PrecisionLandingBase
{
public:
  void ownInit() override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Init start");

    speed_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(node_ptr_);

    node_ptr_->declare_parameter<std::string>("marker_frame_id", "aruco_1");
    node_ptr_->get_parameter("marker_frame_id", marker_frame_id_);

    node_ptr_->declare_parameter<double>("vconstant_descent", 0.5);
    node_ptr_->get_parameter("vconstant_descent", vconstant_descent_);
    node_ptr_->declare_parameter<double>("vconstant_landing_radius", 0.3);
    node_ptr_->get_parameter("vconstant_landing_radius", vconstant_landing_radius_);
    node_ptr_->declare_parameter<double>("vconstant_z_distance_threshold", 0.1);
    node_ptr_->get_parameter("vconstant_z_distance_threshold", vconstant_z_distance_threshold_);
    node_ptr_->declare_parameter<double>("vconstant_xy_gain", 1.0);
    node_ptr_->get_parameter("vconstant_xy_gain", vconstant_xy_gain_);
    node_ptr_->declare_parameter<double>("vconstant_xy_speed_max", 1.0);
    node_ptr_->get_parameter("vconstant_xy_speed_max", vconstant_xy_speed_max_);

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[vconstant] Params -> descent: %.2f | radius: %.2f | z_th: %.2f | xy_gain: %.2f | xy_vmax: %.2f | marker: %s",
                vconstant_descent_, vconstant_landing_radius_, vconstant_z_distance_threshold_,
                vconstant_xy_gain_, vconstant_xy_speed_max_, marker_frame_id_.c_str());

    resetArucoStatus();
  }

  bool own_activate(as2_msgs::action::PrecisionLanding::Goal& _goal) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing accepted");
    resetArucoStatus();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string>&) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing canceled, set to hover");
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string>&) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string>&) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing resumed");
    resetArucoStatus();
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus& state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing end - state: %d", (int)state);
    if (state != as2_behavior::ExecutionStatus::SUCCESS)
      sendHover();
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    geometry_msgs::msg::TransformStamped tf_aruco;
    if (!tryGetArucoTF(tf_aruco)) {
      if (arucoTimeoutExceeded()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "[vconstant] ArUco TF timeout -> failure");
        result_.precision_landing_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
      RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] No ArUco TF yet, hovering...");
      sendHover();
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    const auto [dx, dy, dz] = computeRelativeError(tf_aruco);
    const double dist_xy = std::hypot(dx, dy);
    const double abs_dz  = std::fabs(dz);

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[vconstant] TF OK -> dx=%.2f dy=%.2f dz=%.2f | dist_xy=%.2f | abs_dz=%.2f",
                dx, dy, dz, dist_xy, abs_dz);

    if (abs_dz < vconstant_z_distance_threshold_) {
      handleSuccess(abs_dz);
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    const auto [vx, vy, vz] = computeVelocityCommand(dx, dy, dz, dist_xy);
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[vconstant] Command -> vx=%.2f vy=%.2f vz=%.2f (mode: %s)",
                vx, vy, vz, dist_xy > vconstant_landing_radius_ ? "XY correction" : "Z descent");

    if (!sendSpeedCommand(vx, vy, vz)) {
      result_.precision_landing_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    feedback_.distance_xy = dist_xy;
    feedback_.distance_z  = dz;

    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> speed_motion_handler_{nullptr};

  std::string marker_frame_id_{"aruco"};
  double vconstant_descent_{0.5};
  double vconstant_landing_radius_{0.3};
  double vconstant_z_distance_threshold_{0.1};
  double vconstant_xy_gain_{1.0};
  double vconstant_xy_speed_max_{1.0};

  bool last_aruco_ok_{false};
  rclcpp::Time last_aruco_time_;

private:
  void resetArucoStatus()
  {
    last_aruco_ok_ = false;
    last_aruco_time_ = node_ptr_->now();
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] ArUco status reset");
  }

  bool tryGetArucoTF(geometry_msgs::msg::TransformStamped& tf_out)
  {
    const std::string target_frame = "earth";
    const std::string source_frame = marker_frame_id_;
    try {
      tf_out = tf_handler_->getTransform(target_frame, source_frame);
      last_aruco_ok_ = true;
      last_aruco_time_ = node_ptr_->now();  // atualiza "visto por último"
      RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Got TF (%s -> %s)",
                  source_frame.c_str(), target_frame.c_str());
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), "[vconstant] TF not available (%s->%s): %s",
                   source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  bool arucoTimeoutExceeded() const
  {
    const double elapsed = (node_ptr_->now() - last_aruco_time_).seconds();
    const bool timeout = (elapsed > params_.aruco_timeout_threshold);
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[vconstant] ArUco timeout check -> last_ok=%d | elapsed=%.2f | limit=%.2f | timeout=%d",
                last_aruco_ok_, elapsed, params_.aruco_timeout_threshold, timeout);
    return timeout;
  }

  std::tuple<double, double, double> computeRelativeError(const geometry_msgs::msg::TransformStamped& tf_aruco)
  {
    const auto& p_d = actual_pose_.pose.position;
    const auto& t_a = tf_aruco.transform.translation;
    return {t_a.x - p_d.x, t_a.y - p_d.y, t_a.z - p_d.z};
  }

  void handleSuccess(double abs_dz)
  {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[vconstant] Reached z threshold (|dz|=%.2f < %.2f). Success.",
                abs_dz, vconstant_z_distance_threshold_);
    result_.precision_landing_success = true;
    sendHover();
  }

  std::tuple<double, double, double> computeVelocityCommand(double dx, double dy, double dz, double dist_xy)
  {
    double vx = 0.0, vy = 0.0, vz = 0.0;

    // Correção XY se fora do raio
    if (dist_xy > vconstant_landing_radius_) {
      vx = vconstant_xy_gain_ * dx;
      vy = vconstant_xy_gain_ * dy;
      const double vxy = std::hypot(vx, vy);
      if (vxy > vconstant_xy_speed_max_) {
        const double s = vconstant_xy_speed_max_ / (vxy + 1e-6);
        vx *= s;
        vy *= s;
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[vconstant] XY speed saturated (%.2f > %.2f), scaled by %.2f",
                    vxy, vconstant_xy_speed_max_, s);
      }
      vz = 0.0;
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[vconstant] Outside landing radius, correcting XY only");
    } else {
      // >>> Só desce se o ArUco estiver visível
      if (last_aruco_ok_) {
        vz = -std::fabs(vconstant_descent_);
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[vconstant] Inside radius AND ArUco visible -> descending");
      } else {
        vz = 0.0;
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "[vconstant] Inside radius but ArUco not visible -> holding altitude");
      }
    }

    return {vx, vy, vz};
  }

  bool sendSpeedCommand(double vx, double vy, double vz)
  {
    const bool ok = speed_motion_handler_->sendSpeedCommandWithYawSpeed("earth", vx, vy, vz, 0.0);
    if (!ok) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "[vconstant] Error sending speed command");
    } else {
      RCLCPP_DEBUG(node_ptr_->get_logger(), "[vconstant] Sent speed cmd -> vx=%.2f vy=%.2f vz=%.2f", vx, vy, vz);
    }
    return ok;
  }
};

} // namespace precision_landing_plugin_vconstant

PLUGINLIB_EXPORT_CLASS(precision_landing_plugin_vconstant::Plugin,
                       precision_landing_base::PrecisionLandingBase)

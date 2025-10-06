// BSD-3-Clause
/**
 * @file precision_landing_plugin_vconstant.cpp
 *
 * Plugin: dentro do raio XY, desce a v constante; fora do raio, corrige XY.
 * Usa somente SpeedMotion.
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
    speed_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(node_ptr_);

    // Param extra opcional para tuning do XY
    node_ptr_->declare_parameter<std::string>("marker_frame_id", "aruco");
    node_ptr_->get_parameter("marker_frame_id", marker_frame_id_);

    node_ptr_->declare_parameter<double>("xy_gain", 1.0);      // m/s por m de erro
    node_ptr_->declare_parameter<double>("xy_speed_max", 1.0); // saturação
    node_ptr_->get_parameter("xy_gain", xy_gain_);
    node_ptr_->get_parameter("xy_speed_max", xy_speed_max_);

    last_aruco_ok_ = false;
    last_aruco_time_ = node_ptr_->now();
  }

  bool own_activate(as2_msgs::action::PrecisionLanding::Goal& /*goal*/) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing activated");
    last_aruco_ok_ = false;
    last_aruco_time_ = node_ptr_->now();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string>& /*message*/) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing canceled -> hover");
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string>& /*message*/) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing paused -> hover");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string>& /*message*/) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing resumed");
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus& state) override
  {
    if (state != as2_behavior::ExecutionStatus::SUCCESS) {
      sendHover();
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "[vconstant] Precision Landing end");
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    // 1) Tenta obter TF do aruco em earth
    geometry_msgs::msg::TransformStamped tf_aruco;
    bool have_tf = false;
    try {
      tf_aruco = tf_handler_->lookupTransform(
          "earth", marker_frame_id_, tf2::TimePointZero,
          rclcpp::Duration::from_seconds(params_.tf_timeout_threshold));
      have_tf = true;
      last_aruco_ok_ = true;
      last_aruco_time_ = node_ptr_->now();
    } catch (const tf2::TransformException& ex) {
      have_tf = false;
      (void)ex;
    }

    // 1.a) Timeout do aruco ausente
    if (!have_tf) {
      const double elapsed = (node_ptr_->now() - last_aruco_time_).seconds();
      if (!last_aruco_ok_ || elapsed > params_.aruco_timeout_threshold) {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "[vconstant] ArUco TF timeout (%.2fs) -> failure", elapsed);
        result_.success = false;  // campo típico; ajuste ao seu .action se diferente
        return as2_behavior::ExecutionStatus::FAILURE;
      }
      // Sem TF momentaneamente: mantenha hover
      sendHover();
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    // 2) Erro relativo (drone -> aruco) em earth
    const auto& p_d = actual_pose_.pose.position; // drone em earth
    const auto& t_a = tf_aruco.transform.translation; // aruco em earth

    const double dx = t_a.x - p_d.x;
    const double dy = t_a.y - p_d.y;
    const double dz = t_a.z - p_d.z;

    const double dist_xy = std::hypot(dx, dy);
    const double abs_dz  = std::fabs(dz);

    // 3) Condição de término (altura sobre o aruco)
    if (abs_dz < params_.z_distance_thresshold) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[vconstant] Reached z threshold (|dz|=%.2f < %.2f). Success.",
                  abs_dz, params_.z_distance_thresshold);
      result_.success = true;
      // Para garantir parada
      sendHover();
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    double vx=0.0, vy=0.0, vz=0.0;

    // 4) Controle XY: fora do raio, centraliza
    if (dist_xy > params_.landing_radius) {
      // velocidade proporcional e saturada
      vx = xy_gain_ * dx;
      vy = xy_gain_ * dy;
      const double vxy = std::hypot(vx, vy);
      if (vxy > xy_speed_max_) {
        const double s = xy_speed_max_ / (vxy + 1e-6);
        vx *= s;
        vy *= s;
      }
      vz = 0.0; // não descer até entrar no raio
    } else {
      // 5) Dentro do raio: desce a velocidade constante
      vx = 0.0;
      vy = 0.0;
      // convenção: +z para cima -> descer é velocidade negativa
      vz = -std::fabs(params_.v_constant_descent);
    }

    // 6) Envia comando de velocidade no frame earth
    const bool ok = speed_motion_handler_->sendSpeedCommandWithYawSpeed(
        "earth", vx, vy, vz, 0.0);

    if (!ok) {
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "[vconstant] Error sending speed command");
      result_.success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    // Feedback mínimo (ajuste campos conforme seu .action)
    feedback_.distance_xy = dist_xy;
    feedback_.distance_z  = dz;

    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> speed_motion_handler_{nullptr};

  std::string marker_frame_id_{"aruco"};
  double xy_gain_{1.0};
  double xy_speed_max_{1.0};

  bool   last_aruco_ok_{false};
  rclcpp::Time last_aruco_time_;
};

} // namespace precision_landing_plugin_vconstant

PLUGINLIB_EXPORT_CLASS(precision_landing_plugin_vconstant::Plugin,
                       precision_landing_base::PrecisionLandingBase)

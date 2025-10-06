// Copyright 2024 Universidad Politécnica de Madrid
// BSD-3-Clause

/**
 * @file precision_landing_base.hpp
 *
 * Base class for precision landing plugins header
 */

#ifndef PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BASE_HPP_
#define PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BASE_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "as2_msgs/action/precision_landing.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"

namespace precision_landing_base
{

struct PrecisionLandingParams
{
  double aruco_timeout_threshold   = 10.0; // s
  double landing_radius            = 1.0;  // m
  double v_constant_descent        = 0.5;  // m/s
  double z_distance_thresshold     = 0.3;  // m
  double tf_timeout_threshold      = 0.1;  // s
};

class PrecisionLandingBase
{
public:
  using GoalHandlePrecisionLanding =
      rclcpp_action::ServerGoalHandle<as2_msgs::action::PrecisionLanding>;

  PrecisionLandingBase() {}
  virtual ~PrecisionLandingBase() {}

  void initialize(as2::Node* node_ptr,
                  std::shared_ptr<as2::tf::TfHandler> tf_handler,
                  PrecisionLandingParams& params)
  {
    node_ptr_  = node_ptr;
    tf_handler_ = tf_handler;
    params_    = params;
    hover_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::HoverMotion>(node_ptr_);
    ownInit();
  }

  // Atualização de estado (pose/twist do drone em earth)
  virtual void state_callback(geometry_msgs::msg::PoseStamped& pose_msg,
                              geometry_msgs::msg::TwistStamped& twist_msg)
  {
    actual_pose_  = pose_msg;
    actual_twist_ = twist_msg;
    localization_flag_ = true;
  }

  bool on_activate(std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal)
  {
    auto goal_copy = *goal;
    if (!processGoal(goal_copy)) { return false; }
    if (own_activate(goal_copy)) {
      goal_ = goal_copy;
      return true;
    }
    return false;
  }

  bool on_modify(std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal)
  {
    auto goal_copy = *goal;
    if (!processGoal(goal_copy)) { return false; }
    if (own_modify(goal_copy)) {
      goal_ = goal_copy;
      return true;
    }
    return false;
  }

  inline bool on_deactivate(const std::shared_ptr<std::string>& message)
  {
    return own_deactivate(message);
  }

  inline bool on_pause(const std::shared_ptr<std::string>& message)
  {
    return own_pause(message);
  }

  inline bool on_resume(const std::shared_ptr<std::string>& message)
  {
    return own_resume(message);
  }

  void on_execution_end(const as2_behavior::ExecutionStatus& state)
  {
    localization_flag_ = false;
    own_execution_end(state);
  }

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> /*goal*/,
      std::shared_ptr<as2_msgs::action::PrecisionLanding::Feedback>& feedback_msg,
      std::shared_ptr<as2_msgs::action::PrecisionLanding::Result>&   result_msg)
  {
    auto status = own_run();
    feedback_msg = std::make_shared<as2_msgs::action::PrecisionLanding::Feedback>(feedback_);
    result_msg   = std::make_shared<as2_msgs::action::PrecisionLanding::Result>(result_);
    return status;
  }

protected:
  // Helpers
  inline void sendHover() { hover_motion_handler_->sendHover(); }

  // Interface de plugin
  virtual void ownInit() {}
  virtual bool own_activate(as2_msgs::action::PrecisionLanding::Goal& goal) = 0;

  virtual bool own_modify(as2_msgs::action::PrecisionLanding::Goal& /*goal*/)
  {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "PrecisionLanding goal modification not implemented");
    return false;
  }

  virtual bool own_deactivate(const std::shared_ptr<std::string>& message) = 0;

  virtual bool own_pause(const std::shared_ptr<std::string>& /*message*/)
  {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "PrecisionLanding pause not implemented, try cancel");
    return false;
  }

  virtual bool own_resume(const std::shared_ptr<std::string>& /*message*/)
  {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "PrecisionLanding resume not implemented");
    return false;
  }

  virtual void own_execution_end(const as2_behavior::ExecutionStatus& state) = 0;
  virtual as2_behavior::ExecutionStatus own_run() = 0;

  // Valida pré-condições simples
  bool processGoal(as2_msgs::action::PrecisionLanding::Goal& /*goal*/)
  {
    if (!localization_flag_) {
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "Behavior rejected: no localization");
      return false;
    }
    return true;
  }

protected:
  as2::Node* node_ptr_{nullptr};
  std::shared_ptr<as2::tf::TfHandler> tf_handler_{nullptr};

  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_motion_handler_{nullptr};

  as2_msgs::action::PrecisionLanding::Goal    goal_;
  as2_msgs::action::PrecisionLanding::Feedback feedback_;
  as2_msgs::action::PrecisionLanding::Result   result_;

  PrecisionLandingParams params_;

  geometry_msgs::msg::PoseStamped  actual_pose_;
  geometry_msgs::msg::TwistStamped actual_twist_;
  bool localization_flag_{false};
};

} // namespace precision_landing_base

#endif // PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BASE_HPP_

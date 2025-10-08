// Copyright 2024 Universidad Politécnica de Madrid
// BSD-3-Clause

/**
 * @file precision_landing_base.hpp
 *
 * Base class for precision landing plugins header
 * OK Lucca 08/10
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

struct precision_landing_plugin_params
{
  double aruco_timeout_threshold   = 10.0; // s
  double tf_timeout_threshold   = 10.0; // s
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
                  precision_landing_plugin_params& params)
  {
    node_ptr_  = node_ptr;
    tf_handler_ = tf_handler;
    params_    = params;
    hover_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::HoverMotion>(node_ptr_);
    ownInit();
  }

  virtual void state_callback(geometry_msgs::msg::PoseStamped& pose_msg,
                              geometry_msgs::msg::TwistStamped& twist_msg)
  {
    actual_pose_ = pose_msg;

    feedback_.precision_landing_speed = actual_pose_.pose.position.z;
    feedback_.precision_landing_height = twist_msg.twist.linear.z;
    // TODO: distance_to_target

    localization_flag_ = true;
    return;
  }

  bool on_activate(std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal)
  {
    as2_msgs::action::PrecisionLanding::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) {return false;}

    if (own_activate(goal_candidate)) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  bool on_modify(std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal)
  {
    as2_msgs::action::PrecisionLanding::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) {return false;}

    if (own_modify(goal_candidate)) {
      goal_ = goal_candidate;
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
    return;
  }

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> /*goal*/,
      std::shared_ptr<as2_msgs::action::PrecisionLanding::Feedback>& feedback_msg,
      std::shared_ptr<as2_msgs::action::PrecisionLanding::Result>&   result_msg)
  {
    as2_behavior::ExecutionStatus status = own_run();
    
    feedback_msg = std::make_shared<as2_msgs::action::PrecisionLanding::Feedback>(feedback_);
    result_msg   = std::make_shared<as2_msgs::action::PrecisionLanding::Result>(result_);
    return status;
  }

private:
  bool processGoal(as2_msgs::action::PrecisionLanding::Goal & _goal)
  {
    if (!localization_flag_) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Behavior reject, there is no localization");
      return false;
    }

    return true;
  }

private:
  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_motion_handler_ = nullptr;

  /* Interface with plugin */

protected:
  virtual void ownInit() {}

  virtual bool own_activate(as2_msgs::action::PrecisionLanding::Goal& goal) = 0;

  virtual bool own_modify(as2_msgs::action::PrecisionLanding::Goal& goal)
  {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "PrecisionLanding can not be modified, not implemented");
    return false;
  }

  virtual bool own_deactivate(const std::shared_ptr<std::string>& message) = 0;

  virtual bool own_pause(const std::shared_ptr<std::string>& message)
  {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "PrecisionLanding can not be paused, not implemented, try to cancel it");
    return false;
  }

  virtual bool own_resume(const std::shared_ptr<std::string>& message)
  {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "PrecisionLanding can not be resumed, not implemented");
    return false;
  }

  virtual void own_execution_end(const as2_behavior::ExecutionStatus& state) = 0;
  virtual as2_behavior::ExecutionStatus own_run() = 0;

  inline void sendHover()
  {
    hover_motion_handler_->sendHover();
    return;
  }

  inline float getActualYaw()
  {
    return as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
  }

protected:
  as2::Node* node_ptr_{nullptr};
  std::shared_ptr<as2::tf::TfHandler> tf_handler_{nullptr};

  as2_msgs::action::PrecisionLanding::Goal    goal_;
  as2_msgs::action::PrecisionLanding::Feedback feedback_;
  as2_msgs::action::PrecisionLanding::Result   result_;

  precision_landing_plugin_params params_;
  geometry_msgs::msg::PoseStamped  actual_pose_;
  geometry_msgs::msg::TwistStamped actual_twist_;
  bool localization_flag_{false};
}; // class PrecisionLandingBase
} // namespace precision_landing_base

#endif // PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BASE_HPP_

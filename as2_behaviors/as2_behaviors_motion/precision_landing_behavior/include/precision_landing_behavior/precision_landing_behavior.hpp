// OK Lucca 08/10
#ifndef PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BEHAVIOR_HPP_
#define PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BEHAVIOR_HPP_

#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"
#include "as2_msgs/action/precision_landing.hpp"
#include "as2_msgs/msg/platform_info.hpp"

#include "precision_landing_behavior/precision_landing_base.hpp"

class PrecisionLandingBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::PrecisionLanding>
{
public:
  using GoalHandlePrecisionLanding =
      rclcpp_action::ServerGoalHandle<as2_msgs::action::PrecisionLanding>;
  // using PSME = as2_msgs::msg::PlatformStateMachineEvent; para que? (land_behavior.hpp)

  explicit PrecisionLandingBehavior(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  ~PrecisionLandingBehavior();

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg);

  bool process_goal(
    std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal,
    as2_msgs::action::PrecisionLanding::Goal & new_goal);

  bool on_activate(std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string>& message) override;
  bool on_pause(const std::shared_ptr<std::string>& message) override;
  bool on_resume(const std::shared_ptr<std::string>& message) override;
  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal>& goal,
      std::shared_ptr<as2_msgs::action::PrecisionLanding::Feedback>& feedback_msg,
      std::shared_ptr<as2_msgs::action::PrecisionLanding::Result>& result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus& state) override;

private:
  std::string base_link_frame_id_;
  std::shared_ptr<pluginlib::ClassLoader<precision_landing_base::PrecisionLandingBase>> loader_;
  std::shared_ptr<precision_landing_base::PrecisionLandingBase> precision_landing_plugin_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;  
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
};

#endif  // PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BEHAVIOR_HPP_

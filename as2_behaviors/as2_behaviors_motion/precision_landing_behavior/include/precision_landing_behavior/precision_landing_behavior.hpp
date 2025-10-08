// OK Lucca 08/10
#ifndef PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BEHAVIOR_HPP_
#define PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BEHAVIOR_HPP_

#include <string>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/precision_landing.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"
#include "precision_landing_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"

class PrecisionLandingBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::PrecisionLanding>
{
public:
  using GoalHandlePrecisionLanding =
      rclcpp_action::ServerGoalHandle<as2_msgs::action::PrecisionLanding>;
  
  using PSME = as2_msgs::msg::PlatformStateMachineEvent;

  explicit PrecisionLandingBehavior(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  ~PrecisionLandingBehavior();

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg);
  
  bool sendEventFSME(const int8_t _event);

  bool sendDisarm();
  
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
  as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
    platform_land_cli_;
  as2::SynchronousServiceClient<std_srvs::srv::SetBool>::SharedPtr platform_disarm_cli_;
};

#endif  // PRECISION_LANDING_BEHAVIOR__PRECISION_LANDING_BEHAVIOR_HPP_

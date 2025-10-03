#ifndef CENTRALIZE_MARKER_BEHAVIOR__CENTRALIZE_MARKER_BEHAVIOR_HPP_
#define CENTRALIZE_MARKER_BEHAVIOR__CENTRALIZE_MARKER_BEHAVIOR_HPP_

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
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "as2_msgs/action/centralize_marker.hpp"   // FAZER
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"
#include "rclcpp/rclcpp.hpp"

class CentralizeMarkerBehavior 
  : public as2_behavior::BehaviorServer<as2_msgs::action::CentralizeMarker>
{
public:
  using GoalHandleCentralize = 
    rclcpp_action::ServerGoalHandle<as2_msgs::action::CentralizeMarker>;

  explicit CentralizeMarkerBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CentralizeMarkerBehavior();

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);

  bool process_goal(
    std::shared_ptr<const as2_msgs::action::CentralizeMarker::Goal> goal,
    as2_msgs::action::CentralizeMarker::Goal & new_goal);

  bool on_activate(std::shared_ptr<const as2_msgs::action::CentralizeMarker::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::CentralizeMarker::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::CentralizeMarker::Goal> & goal,
    std::shared_ptr<as2_msgs::action::CentralizeMarker::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::CentralizeMarker::Result> & result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  inline void sendHover();

  inline float getActualYaw();

  bool getState();

  bool computeYaw(
    const uint8_t yaw_mode,
    const geometry_msgs::msg::Point & target,
    const geometry_msgs::msg::Point & actual,
    float & yaw);

  bool checkGoal(as2_msgs::action::CentralizeMarker::Goal & _goal);
private:
  std::string base_link_frame_id_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
    platform_event_cli_;
  as2::SynchronousServiceClient<std_srvs::srv::SetBool>::SharedPtr platform_disarm_cli_;

  // Motion handler (use para mandar setpoints simples — por exemplo hover/velocity)
  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_motion_handler_;

  // Internal status & feedback/result
  as2_msgs::action::CentralizeMarkerBehavior::Goal goal_;
  as2_msgs::action::CentralizeMarkerBehavior::Feedback feedback_;
  as2_msgs::action::CentralizeMarkerBehavior::Result result_;

  // Flag que indica se temos localization disponível via TF
  bool localization_flag_;
  geometry_msgs::msg::PoseStamped actual_pose_;
};
#endif  // CENTRALIZE_MARKER_BEHAVIOR__CENTRALIZE_MARKER_BEHAVIOR_HPP_

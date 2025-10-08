// OK Lucca 08/10

#include "precision_landing_behavior/precision_landing_behavior.hpp"

#include "as2_core/core_functions.hpp"
#include "rclcpp_components/register_node_macro.hpp"

PrecisionLandingBehavior::PrecisionLandingBehavior(const rclcpp::NodeOptions& options)
  : as2_behavior::BehaviorServer<as2_msgs::action::PrecisionLanding>(
        as2_names::actions::behaviors::precisionlanding, options)
{
  try {
    this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <plugin_name> not defined or "
      "malformed: %s",
      e.what());
    this->~PrecisionLandingBehavior();
  }
  try {
    this->declare_parameter<std::string>("aruco_timeout_threshold");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <aruco_timeout_threshold> not defined or "
      "malformed: %s",
      e.what());
    this->~PrecisionLandingBehavior();
  }
  try {
    this->declare_parameter<double>("tf_timeout_threshold");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <tf_timeout_threshold> not defined or "
      "malformed: %s",
      e.what());
    this->~PrecisionLandingBehavior();
  }

  loader_ = std::make_shared<pluginlib::ClassLoader<precision_landing_base::PrecisionLandingBase>>(
    "precision_landing_behavior",
    "precision_landing_base::PrecisionLandingBase");

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  try {
    std::string plugin_name = this->get_parameter("plugin_name").as_string();
    plugin_name += "::Plugin";
    precision_landing_plugin_ = loader_->createSharedInstance(plugin_name);

    precision_landing_base::precision_landing_plugin_params params;
    params.aruco_timeout_threshold = this->get_parameter("aruco_timeout_threshold").as_double();
    params.tf_timeout_threshold = this->get_parameter("tf_timeout_threshold").as_double();

    precision_landing_plugin_->initialize(this, tf_handler_, params);
    RCLCPP_INFO(this->get_logger(), "PRECISION LANDING BEHAVIOR PLUGIN LOADED: %s", plugin_name.c_str());
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
      ex.what());
    this->~PrecisionLandingBehavior();
  }

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  platform_disarm_cli_ = std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::SetBool>>(
    as2_names::services::platform::set_arming_state, this);

  platform_land_cli_ =
    std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>>(
    as2_names::services::platform::set_platform_state_machine_event, this);
  
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
    std::bind(&PrecisionLandingBehavior::state_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "PrecisionLanding Behavior ready!");
}

PrecisionLandingBehavior::~PrecisionLandingBehavior() {}

void PrecisionLandingBehavior::state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  try {
    auto [pose_msg, twist_conv] = 
      tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);
    precision_landing_plugin_->state_callback(pose_msg, twist_conv);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

bool PrecisionLandingBehavior::sendEventFSME(const int8_t _event)
{
  as2_msgs::srv::SetPlatformStateMachineEvent::Request set_platform_fsm_req;
  as2_msgs::srv::SetPlatformStateMachineEvent::Response set_platform_fsm_resp;
  set_platform_fsm_req.event.event = _event;
  auto out = platform_land_cli_->sendRequest(set_platform_fsm_req, set_platform_fsm_resp, 3);
  if (out && set_platform_fsm_resp.success) {return true;}
  return false;
}

bool PrecisionLandingBehavior::sendDisarm()
{
  RCLCPP_INFO(this->get_logger(), "Disarming platform");
  std_srvs::srv::SetBool::Request set_platform_disarm_req;
  std_srvs::srv::SetBool::Response set_platform_disarm_resp;
  set_platform_disarm_req.data = false;
  auto out =
    platform_disarm_cli_->sendRequest(set_platform_disarm_req, set_platform_disarm_resp, 3);
  if (out && set_platform_disarm_resp.success) {return true;}
  return false;
}

bool PrecisionLandingBehavior::process_goal(
  std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal,
  as2_msgs::action::PrecisionLanding::Goal & new_goal)
{
  /**
  * Precisa melhorar isso. A action precisa ser recusada se não tiver um TF
  * válido para o último marcador
  */
  return true;
}

bool PrecisionLandingBehavior::on_activate(
    std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal)
{
  as2_msgs::action::PrecisionLanding::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return precision_landing_plugin_->on_activate(std::make_shared<const as2_msgs::action::PrecisionLanding::Goal>(new_goal));
}

bool PrecisionLandingBehavior::on_modify(
    std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal)
{
  as2_msgs::action::PrecisionLanding::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return precision_landing_plugin_->on_modify(std::make_shared<const as2_msgs::action::PrecisionLanding::Goal>(new_goal));
}

bool PrecisionLandingBehavior::on_deactivate(const std::shared_ptr<std::string>& message)
{
  return precision_landing_plugin_->on_deactivate(message);
}

bool PrecisionLandingBehavior::on_pause(const std::shared_ptr<std::string>& message)
{
  return precision_landing_plugin_->on_pause(message);
}

bool PrecisionLandingBehavior::on_resume(const std::shared_ptr<std::string>& message)
{
  return precision_landing_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus PrecisionLandingBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> & goal,
  std::shared_ptr<as2_msgs::action::PrecisionLanding::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::PrecisionLanding::Result> & result_msg)
{
  return precision_landing_plugin_->on_run(goal, feedback_msg, result_msg);
}

void PrecisionLandingBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "PrecisionLandingBehavior: Land successful");
    if (!sendDisarm()) {
      RCLCPP_ERROR(this->get_logger(), "PrecisionLandingBehavior: Could not disarm");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "PrecisionLandingBehavior: Land failed");
  }
  return precision_landing_plugin_->on_execution_end(state);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(PrecisionLandingBehavior)
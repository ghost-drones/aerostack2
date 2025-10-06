#include "precision_landing_behavior/precision_landing_behavior.hpp"

#include "as2_core/core_functions.hpp"
#include "rclcpp_components/register_node_macro.hpp"

PrecisionLandingBehavior::PrecisionLandingBehavior(const rclcpp::NodeOptions& options)
  : as2_behavior::BehaviorServer<as2_msgs::action::PrecisionLanding>(
        // nome da ação do comportamento
        as2_names::actions::behaviors::precision_landing, options)
{
  // Parâmetros principais (defaults conforme seu YAML)
  this->declare_parameter<double>("aruco_timeout_threshold", 10.0);
  this->declare_parameter<double>("landing_radius", 1.0);
  this->declare_parameter<double>("v_constant_descent", 0.5);
  this->declare_parameter<double>("z_distance_thresshold", 0.3);
  this->declare_parameter<double>("tf_timeout_threshold", 0.1);

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  // Estado (pose/twist) a partir do tópico de self-localization
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist,
      as2_names::topics::self_localization::qos,
      std::bind(&PrecisionLandingBehavior::state_callback, this, std::placeholders::_1));

  platform_info_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
      as2_names::topics::platform::info, as2_names::topics::platform::qos,
      std::bind(&PrecisionLandingBehavior::platform_info_callback, this, std::placeholders::_1));

  // Carrega plugin
  loader_ = std::make_shared<pluginlib::ClassLoader<precision_landing_base::PrecisionLandingBase>>(
      "precision_landing_behavior",
      "precision_landing_base::PrecisionLandingBase");

  try {
    plugin_ = loader_->createSharedInstance("precision_landing_plugin_vconstant/Plugin");
  } catch (const pluginlib::PluginlibException& ex) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load precision landing plugin: %s", ex.what());
    throw;
  }

  precision_landing_base::PrecisionLandingParams params;
  params.aruco_timeout_threshold = this->get_parameter("aruco_timeout_threshold").as_double();
  params.landing_radius          = this->get_parameter("landing_radius").as_double();
  params.v_constant_descent      = this->get_parameter("v_constant_descent").as_double();
  params.z_distance_thresshold   = this->get_parameter("z_distance_thresshold").as_double();
  params.tf_timeout_threshold    = this->get_parameter("tf_timeout_threshold").as_double();

  plugin_->initialize(this, tf_handler_, params);

  RCLCPP_DEBUG(this->get_logger(), "PrecisionLanding Behavior ready!");
}

PrecisionLandingBehavior::~PrecisionLandingBehavior() {}

void PrecisionLandingBehavior::state_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
{
  try {
    auto [pose_msg, twist_conv] = tf_handler_->getState(
        *twist_msg, "earth", "earth", base_link_frame_id_);
    // entrega estado ao plugin
    plugin_->state_callback(pose_msg, twist_conv);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
}

void PrecisionLandingBehavior::platform_info_callback(
    const as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  (void)msg; // mantenha caso queira usar no futuro
}

bool PrecisionLandingBehavior::on_activate(
    std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal)
{
  return plugin_->on_activate(goal);
}

bool PrecisionLandingBehavior::on_modify(
    std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal> goal)
{
  return plugin_->on_modify(goal);
}

bool PrecisionLandingBehavior::on_deactivate(const std::shared_ptr<std::string>& message)
{
  return plugin_->on_deactivate(message);
}

bool PrecisionLandingBehavior::on_pause(const std::shared_ptr<std::string>& message)
{
  return plugin_->on_pause(message);
}

bool PrecisionLandingBehavior::on_resume(const std::shared_ptr<std::string>& message)
{
  return plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus PrecisionLandingBehavior::on_run(
    const std::shared_ptr<const as2_msgs::action::PrecisionLanding::Goal>& goal,
    std::

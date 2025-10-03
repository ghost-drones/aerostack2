#include "centralize_marker/centralize_marker_behavior.hpp"

CentralizeMarkerBehavior::CentralizeMarkerBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::CentralizeMarker>(
    as2_names::actions::behaviors::centralize_marker, options),
  localization_flag_(false)
{
  // PARAMETROS
  try {
    this->declare_parameter<double>("centralize_timeout", 30.0);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(this->get_logger(), "Launch argument <centralize_timeout> not defined or malformed: %s",
    e.what());
    this->~CentralizeMarkerBehavior();
  }

  speed_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::PositionMotion>(this);

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  // clients para eventos da plataforma (opcional, keep parity com LandBehavior)
  platform_event_cli_ =
    std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>>(
      as2_names::services::platform::set_platform_state_machine_event, this);

  platform_disarm_cli_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::SetBool>>(
      as2_names::services::platform::set_arming_state, this);

  // Motion handler: usado para enviar referências simples (ex: manter hover enquanto centraliza)
  hover_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::HoverMotion>(this);

  // subscrição do estado (mesmo padrão do land)
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
    std::bind(&CentralizeMarkerBehavior::state_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "CentralizeMarkerBehavior Behavior ready!");
}

CentralizeMarkerBehavior::~CentralizeMarkerBehavior() {}

void CentralizeMarkerBehavior::state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  try {
    auto [pose_msg, twist_msg] =
      tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);
    // Guarda pose atual e atualiza feedback
    actual_pose_ = pose_msg;
    feedback_.current_distance_to_marker = feedback_.current_distance_to_marker; // placeholder
    // Se você tiver um módulo de visão que fornece a distância ao marker, atualize feedback_ aqui.
    localization_flag_ = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

bool CentralizeMarkerBehavior::sendEventFSME(const int8_t _event)
{
  as2_msgs::srv::SetPlatformStateMachineEvent::Request req;
  as2_msgs::srv::SetPlatformStateMachineEvent::Response resp;
  req.event.event = _event;
  auto out = platform_event_cli_->sendRequest(req, resp, 3);
  if (out && resp.success) {return true;}
  return false;
}

bool CentralizeMarkerBehavior::sendDisarm()
{
  RCLCPP_INFO(this->get_logger(), "Disarming platform");
  std_srvs::srv::SetBool::Request req;
  std_srvs::srv::SetBool::Response resp;
  req.data = false;
  auto out = platform_disarm_cli_->sendRequest(req, resp, 3);
  if (out && resp.success) {return true;}
  return false;
}

bool CentralizeMarkerBehavior::process_goal(
  std::shared_ptr<const as2_msgs::action::Centralize_marker::Goal> goal,
  as2_msgs::action::Centralize_marker::Goal & new_goal)
{
  // Check localization available
  if (!localization_flag_) {
    RCLCPP_ERROR(this->get_logger(), "CentralizeBehavior: reject, there is no localization");
    return false;
  }

  // Exemplos de processamento do goal: timeout, tolerância, id do marcador, etc.
  // Como assumido: não há plugins e não há setpoints diferenciados.
  // Apenas copie os parâmetros do goal (se existirem) para new_goal.
  new_goal = *goal;

  // opcional: acionar FSM para modo "centralize"
  // sendEventFSME(PSME::CENTRALIZE); // descomente se tiver evento

  return true;
}

bool CentralizeMarkerBehavior::on_activate(std::shared_ptr<const as2_msgs::action::Centralize_marker::Goal> goal)
{
  as2_msgs::action::Centralize_marker::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }

  // Inicie qualquer estado interno necessário
  goal_ = new_goal;
  feedback_ = as2_msgs::action::Centralize_marker::Feedback();
  result_ = as2_msgs::action::Centralize_marker::Result();

  RCLCPP_INFO(this->get_logger(), "CentralizeMarkerBehavior: activated");
  return true;
}

bool CentralizeMarkerBehavior::on_modify(std::shared_ptr<const as2_msgs::action::Centralize_marker::Goal> goal)
{
  as2_msgs::action::Centralize_marker::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  goal_ = new_goal;
  RCLCPP_INFO(this->get_logger(), "CentralizeMarkerBehavior: modified");
  return true;
}

bool CentralizeMarkerBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "CentralizeMarkerBehavior: deactivated: %s", message->c_str());
  return true;
}

bool CentralizeMarkerBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "CentralizeMarkerBehavior: paused: %s", message->c_str());
  return true;
}

bool CentralizeMarkerBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "CentralizeMarkerBehavior: resumed: %s", message->c_str());
  return true;
}

as2_behavior::ExecutionStatus CentralizeMarkerBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::Centralize_marker::Goal> & goal,
  std::shared_ptr<as2_msgs::action::Centralize_marker::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::Centralize_marker::Result> & result_msg)
{
  // Exemplo de lógica de execução (skeleton):
  // 1) receba informação de visão (proporção/erro do marker em imagem)
  // 2) converta erro de imagem em setpoint/velocidade
  // 3) envie referencia via motion handler
  // 4) atualize feedback e decida SUCCESS/FAIL/RUNNING

  // PLACEHOLDERS: o usuário deve substituir por integração com seu detector de marker.
  bool marker_detected = false;          // <- integrar detector
  double lateral_error = 0.0;            // <- integrar detector (m ou pixels -> converter)
  double distance_to_marker = 999.0;     // <- integrar detector / lidar

  // EXEMPLO de uso do motion handler (manter hover enquanto centraliza):
  hover_motion_handler_->sendHover();

  // Atualiza feedback
  feedback_.current_distance_to_marker = distance_to_marker;
  feedback_.lateral_error = lateral_error;
  feedback_msg = std::make_shared<as2_msgs::action::Centralize_marker::Feedback>(feedback_);

  // Critérios simplificados:
  if (!marker_detected) {
    // ainda buscando: continuar
    result_msg = std::make_shared<as2_msgs::action::Centralize_marker::Result>(result_);
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  // se detectado e dentro da tolerância:
  const double tolerance = (goal_->tolerance > 0.0) ? goal_->tolerance : 0.05;
  if (std::fabs(lateral_error) <= tolerance) {
    // chegou: sucesso
    result_.success = true;
    result_msg = std::make_shared<as2_msgs::action::Centralize_marker::Result>(result_);
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  // caso contrário, envie velocidades proporcionais (exemplo)
  // -> aqui você converteria lateral_error em referência de velocidade/position
  // hover_motion_handler_->sendSetpoint(...);

  result_msg = std::make_shared<as2_msgs::action::Centralize_marker::Result>(result_);
  return as2_behavior::ExecutionStatus::RUNNING;
}

void CentralizeMarkerBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "CentralizeMarkerBehavior: Success");
    // opcional: enviar evento FSM para "centralized" ou "land" etc.
  } else {
    RCLCPP_WARN(this->get_logger(), "CentralizeMarkerBehavior: Failed or Cancelled");
    // tratamento de erro (opcional)
  }
  // sempre encerrar limpando flags internas
  localization_flag_ = false;
  return;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CentralizeMarkerBehavior)

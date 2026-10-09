#include "decision_maker_node.hpp"
using namespace drn_navigation_msgs::msg;  // Add this line for convenience

DecisionMakerNode::DecisionMakerNode()
    : Node("decision_maker_node"),
      shared_data_(SharedData::getInstance()) {
    
    // Initialize ROS2 communications
    mission_sub_ = create_subscription<drn_integration_msgs::msg::Mission>(
        "/mission", 10,
        std::bind(&DecisionMakerNode::missionCallback, this, std::placeholders::_1));
        
    // status_pub_ = create_publisher<RobotStatus>("/robot_status", 10);
    ls_config_pub_ = create_publisher<std_msgs::msg::Int32>("/ls_config", 10);

    status_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DecisionMakerNode::publishStatus, this));

    // sensors_sub_ = create_subscription<frequency_check_msg::msg::FrequencyCheck>("/frequency_check", 10, std::bind(&DecisionMakerNode::sensorErrorCallback, this, std::placeholders::_1));
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10, std::bind(&DecisionMakerNode::poseCallback, this, std::placeholders::_1));
    // initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 10, std::bind(&DecisionMakerNode::poseCallback, this, std::placeholders::_1));
    mission_log_client_ = create_client<drn_integration_msgs::srv::MissionLog>("/drn_integration/communication/mission_log_service");
    mission_response_client_ = create_client<drn_integration_msgs::srv::Mission>("/drn_integration/communication/mission_service");
    alert_client_ = create_client<drn_integration_msgs::srv::Alert>("/drn_integration/communication/alert_service");
    cancel_mission_client_ = create_client<action_msgs::srv::CancelGoal>("/navigate_through_poses/_action/cancel_goal");
    plc_msg_sub_ = create_subscription<siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>("/siemens_et200sp", 10, std::bind(&DecisionMakerNode::modeCallback, this, std::placeholders::_1));

    this->declare_parameter("map", "[]");
    this->declare_parameter("vehicle", "[]");
    this->declare_parameter("setting/dockDetectionAttempts", "3");

    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DecisionMakerNode::onParameterChange, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult DecisionMakerNode::onParameterChange(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto& param : parameters) {
        if (param.get_name() == "map") {
            RCLCPP_INFO(this->get_logger(), "Parâmetro 'map' atualizado: %s", param.as_string().c_str());
        } else if (param.get_name() == "vehicle") {
            RCLCPP_INFO(this->get_logger(), "Parâmetro 'vehicle' atualizado: %s", param.as_string().c_str());
        } else if (param.get_name() == "setting/dockDetectionAttempts") {
            try {
                shared_data_->setting_dock_detection_attempts_param = std::stoi(param.as_string());
                RCLCPP_INFO(this->get_logger(), "Parâmetro 'dockDetectionAttempts' atualizado: %d", shared_data_->setting_dock_detection_attempts_param);
            } catch (const std::exception& e) {
                result.successful = false;
                result.reason = "Valor inválido para 'setting/dockDetectionAttempts'";
                RCLCPP_ERROR(this->get_logger(), "Erro ao atualizar 'dockDetectionAttempts': %s", e.what());
            }
        }
    }

    current_state_->exit();
    current_state_ = state::State::createState(state::State::StateType::OFF, shared_from_this());
    current_state_->enter();
    return result;
}


void DecisionMakerNode::initialize() {
    // mission_manager_ = std::make_shared<MissionManager>(shared_from_this());
    current_state_ = state::State::createState(state::State::StateType::OFF, shared_from_this());
    current_state_->enter();
}

void DecisionMakerNode::run() {
    while (rclcpp::ok()) {
        if (current_state_ && !shared_data_->robot_pause) {
            current_state_->execute();
            // current_state_->exit();
            auto next_state = current_state_->getCurrentState();
            
            ls_config_pub_->publish(shared_data_->ls_config);
            if (next_state && next_state != current_state_) {
                current_state_->exit();
                current_state_ = next_state;
                current_state_->enter();
            }
        }
        
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void DecisionMakerNode::missionCallback(const drn_integration_msgs::msg::Mission::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "[MISSION CALLBACK] Received mission command: %s", msg->command.c_str());
    
    if (!shared_data_->robot_auto_mode) {
        // Missão não iniciada pois o AMR não está em modo autônomo, TODO: enviar log ou log de missão
        RCLCPP_INFO(get_logger(), "[MISSION CALLBACK] Command ignored, robot in manual mode");
        this->sendMissionResponse(msg->command, false, msg->id);
        return;
    } else if (msg->command == mission_response::startMission) {
        if (shared_data_->active_mission) {
            // Missão não iniciada pois o AMR possui outra missão em execução, TODO: enviar log ou log de missão
            this->sendMissionResponse(mission_response::startMission, false, msg->id);
            return;
        }
        RCLCPP_INFO(get_logger(), "[MISSION CALLBACK] Executing mission %d type: %s", msg->end_id, msg->type.c_str());
        shared_data_->resetMemory();
        shared_data_->new_mission = true;
        shared_data_->active_mission = true;
        shared_data_->mission_id = msg->id;
        shared_data_->mission_end_id = msg->end_id;
        shared_data_->mission_type = msg->type;
        shared_data_->mission_direction = msg->mission_direction;
        this->sendMissionLog(shared_data_->mission_id, "INICIADA", "Missão iniciada");
        this->sendMissionResponse(mission_response::startMission, true, msg->id);
    } else if (msg->command == mission_response::cancelMission) {
        if (!shared_data_->active_mission) {
            // Missão não foi pausada pois não há missão em execução
            this->sendMissionResponse(mission_response::cancelMission, false, msg->id);
            return;
        }
        RCLCPP_INFO(get_logger(), "Mission cancelled");
        shared_data_->resetMemory();
        shared_data_->mission_canceled = true;
        this->cancelMissionService();
        this->sendMissionLog(shared_data_->mission_id, "CANCELADA", "Missão cancelada");
        this->sendMissionResponse(mission_response::cancelMission, true, msg->id);
        // current_state_ = state::State::createState(state::State::StateType::IDLE, shared_from_this());
    } else if (msg->command ==mission_response::pauseMission && shared_data_->active_mission && !shared_data_->robot_pause) {
        // Placeholder de falha no comando de pause da missão, apenas para não quebrar nada, funcionalidade não implementada do lado da tomada de decisão
        this->sendMissionResponse(mission_response::pauseMission, false, msg->id);
        return;

        RCLCPP_INFO(get_logger(), "Mission paused");
        shared_data_->robot_pause = true;
        this->sendMissionLog(shared_data_->mission_id, "PAUSADA", "Missão pausada");
        this->sendMissionResponse(mission_response::pauseMission, true, msg->id);
    } else if (msg->command == mission_response::continueMission && shared_data_->robot_pause) {
        // Placeholder de falha no comando de pause da missão, apenas para não quebrar nada, funcionalidade não implementada do lado da tomada de decisão
        this->sendMissionResponse(mission_response::continueMission, false, msg->id);
        return;
        
        RCLCPP_INFO(get_logger(), "Mission resumed");
        shared_data_->robot_pause = false;
        this->sendMissionLog(shared_data_->mission_id, "CONTINUANDO", "Missão retomada");
        this->sendMissionResponse(mission_response::continueMission, true, msg->id);
    }
}

void DecisionMakerNode::modeCallback(const siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "PLC MSG received");

    // XOR: verdadeiro se um for true e o outro false
    if (!(msg->auto_sw ^ msg->man_sw) ) {
        // Situação inválida: ambos true ou ambos false
        RCLCPP_ERROR(this->get_logger(), "Modos conflitantes: auto_sw = %s, man_sw = %s, robô ficara em estado [MANUAL]",
            msg->auto_sw ? "true" : "false",
            msg->man_sw ? "true" : "false");
        return;
    } else if (shared_data_->robot_auto_mode != msg->auto_sw) {
        shared_data_->robot_auto_mode = msg->auto_sw;  // auto = true → modo automático; man = true → modo manual
        RCLCPP_INFO(this->get_logger(), "Mudança de modo detectada, robot_mode = %s", shared_data_->robot_auto_mode ? "[AUTOMÁTICO]" : "[MANUAL]");
        if (!shared_data_->robot_auto_mode) {
            if (shared_data_->active_mission) {
            RCLCPP_INFO(get_logger(), "Mission cancelled");
            shared_data_->resetMemory();
            shared_data_->mission_canceled = true;
            this->cancelMissionService();
            this->sendMissionLog(shared_data_->mission_id, "CANCELADA", "Missão cancelada pois AMR entrou em modo manual");
            this->sendMissionResponse(mission_response::finishMission, false, shared_data_->mission_id);
            }
            if (current_state_->getType() != state::State::StateType::OFF) {
                current_state_->exit();
                current_state_ = state::State::createState(state::State::StateType::OFF, shared_from_this());
                current_state_->enter();
            }
        }
    }
}

void DecisionMakerNode::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // RCLCPP_INFO(get_logger(), "Pose recebida: %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    shared_data_->robot_pose.header = msg->header;
    shared_data_->robot_pose.pose = msg->pose.pose;
}

// void DecisionMakerNode::sensorErrorCallback(const frequency_check_msg::msg::FrequencyCheck::SharedPtr msg){
//     ErrorCode inconsistent_sensor_error = ErrorCode::INCONSISTENT_SENSOR;
//     auto it = shared_data_->alert_map.find(inconsistent_sensor_error);
//     auto now = std::chrono::system_clock::now();
//     std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
//     std::string data = error_code_description.at(inconsistent_sensor_error);
//     std::string message = "Leitura inconsistente de sensor: ";
//     if (msg->alert){
//         if (shared_data_->alert_map.find(inconsistent_sensor_error) == shared_data_->alert_map.end()) {
//             std::string uuid = std::to_string(timestamp);
//             this->sendAlert(uuid, true, static_cast<int>(inconsistent_sensor_error), data, message + msg->frequency_error, std::to_string(timestamp));
//             shared_data_->alert_map[inconsistent_sensor_error] = uuid;
//             RCLCPP_INFO(get_logger(), "Inconsistent sensor alert!");
//         }
//     }
//     else if (it != shared_data_->alert_map.end()){
//         std::string uuid = it->second;
//         this->sendAlert(uuid, false, static_cast<int>(inconsistent_sensor_error), data, message, std::to_string(timestamp));
//         shared_data_->alert_map.erase(it);
//         RCLCPP_INFO(get_logger(), "Inconsistent sensor alert solved!");
//     }
// }

void DecisionMakerNode::publishStatus() {
    auto msg = std::make_unique<RobotStatus>();
    msg->state = static_cast<uint8_t>(current_state_->getType());
    msg->battery_level = shared_data_->battery_level;
    msg->is_charging = shared_data_->is_charging;
    msg->mission_id = shared_data_->mission_id;
    msg->is_active = shared_data_->active_mission;
    
    // status_pub_->publish(std::move(msg));
}

void DecisionMakerNode::sendMissionLog(int id, const std::string& status, const std::string& log_message){

    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

    auto request = std::make_shared<drn_integration_msgs::srv::MissionLog::Request>();
    
    request->uuid = std::to_string(timestamp);
    request->id = id;
    request->status = status;
    request->data = log_message;
    request->timestamp = std::to_string(timestamp);
    // auto result_future = mission_log_client_->async_send_request(request);
    auto result_future = mission_log_client_->async_send_request(request, 
        [this](rclcpp::Client<drn_integration_msgs::srv::MissionLog>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "Mission log sent: %s", response->success ? "True" : "False");
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Erro ao processar resposta do serviço: %s", e.what());
            }
        });

    RCLCPP_INFO(get_logger(), "Solicitação de log de missão enviada!");

}

void DecisionMakerNode::sendMissionResponse(const std::string& command, bool status, int id){

    auto request = std::make_shared<drn_integration_msgs::srv::Mission::Request>();
    
    request->command = command;
    request->status = status;
    request->id = id;

    auto result_future = mission_response_client_->async_send_request(request, 
        [this](rclcpp::Client<drn_integration_msgs::srv::Mission>::SharedFuture future) {
            try {
                auto response = future.get();
                // RCLCPP_INFO(get_logger(), "Mission response sent: %s", response->success ? "True" : "False");
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Erro ao processar resposta do serviço: %s", e.what());
            }
        });
}

void DecisionMakerNode::sendAlert(const std::string& uuid, bool active, int errorcode, const std::string& data, const std::string& message, const std::string& timestamp){
    auto request = std::make_shared<drn_integration_msgs::srv::Alert::Request>();
    
    request->uuid = uuid;
    request->active = active;
    request->errorcode = errorcode;
    request->data = data;
    request->message = message;
    request->timestamp = timestamp;
    auto result_future = alert_client_->async_send_request(request, 
        [this](rclcpp::Client<drn_integration_msgs::srv::Alert>::SharedFuture future) {
            try {
                auto response = future.get();
                // RCLCPP_INFO(get_logger(), "Alert response sent: %s", response->success ? "True" : "False");
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Erro ao processar resposta do serviço: %s", e.what());
            }
        });
}
void DecisionMakerNode::cancelMissionService() {
    while (!cancel_mission_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "Aguardando o serviço de cancelamento de missão...");
    }

    auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

    request->goal_info.goal_id.uuid.fill(0);

    auto result_future = cancel_mission_client_->async_send_request(request, 
        [this](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future) {
            try {
                auto response = future.get();
                if (!response->goals_canceling.empty()) {
                    RCLCPP_INFO(get_logger(), "Trajetória cancelada com sucesso!");
                } else {
                    RCLCPP_WARN(get_logger(), "Nenhuma trajetória estava ativa para ser cancelada.");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Erro ao cancelar trajetória: %s", e.what());
            }
        });

    RCLCPP_INFO(get_logger(), "Solicitação de cancelamento enviada!");
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionMakerNode>();
    node->initialize();
    node->run();
    rclcpp::shutdown();
    return 0;
}
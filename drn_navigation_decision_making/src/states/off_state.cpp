#include <states/off_state.hpp>
// #include <states/configuration_state.hpp>

namespace state {

OffState::OffState(std::shared_ptr<rclcpp::Node> node) : State(node) {
    get_parameters_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/drn_integration/communication/drn_integration_communication_node/get_parameters");
}

void OffState::enter() {
    RCLCPP_INFO(node_->get_logger(), "Entering OFF state");
    this->shared_data_->robot_status = false;
    if (!shared_data_->robot_auto_mode) {
        RCLCPP_INFO(node_->get_logger(), "Robot in manual mode, waiting...");
    }
}

void OffState::execute() {
    if (!this->shared_data_->robot_auto_mode) {
        return;
    }
    // RCLCPP_INFO(node_->get_logger(), "Executing OFF state");

    this->checkAndHandleAlert(ErrorCode::UNREGISTERED_ROBOT, !this->isRobotRegistered() || !this->getSettings(), "Configuração não encontrada");
    if (this->shared_data_->alert_map.find(ErrorCode::UNREGISTERED_ROBOT) != this->shared_data_->alert_map.end()) {
        return;
    }

    this->checkAndHandleAlert(ErrorCode::UNCONFIGURED_MAP, !this->isMapRegistered(), "O robô não possui um mapa configurado.");
    if (this->shared_data_->alert_map.find(ErrorCode::UNCONFIGURED_MAP) != this->shared_data_->alert_map.end()) {
        return;
    }

    next_state_ = transition(StateType::IDLE);
}

void OffState::exit() {
    RCLCPP_INFO(node_->get_logger(), "Exiting OFF state");
    this->shared_data_->robot_status = true;
}

bool OffState::isRobotRegistered() {
    static std::optional<bool> last_status;
    bool current_status = false;

    if (node_->get_parameter("vehicle", shared_data_->vehicle_param_)) {
        try {
            nlohmann::json jsonData = nlohmann::json::parse(shared_data_->vehicle_param_);
            std::string vehicle_code = jsonData[0]["code"];
            int vehicle_id = jsonData[0]["id"];
            std::string vehicle_model = jsonData[0]["model"];

            current_status = true;
            if (last_status != current_status) {
                RCLCPP_INFO(node_->get_logger(), "vehicle: %s, id: %d, model: %s", vehicle_code.c_str(), vehicle_id, vehicle_model.c_str());
            }
        } catch (...) {
            current_status = false;
            if (last_status != current_status) {
                RCLCPP_ERROR(node_->get_logger(), "Erro ao interpretar JSON do parâmetro 'vehicle'.");
            }
        }
    } else {
        current_status = false;
        if (last_status != current_status) {
            RCLCPP_ERROR(node_->get_logger(), "Parâmetro 'vehicle' não encontrado.");
        }
    }

    last_status = current_status;
    return current_status;
}

bool OffState::isMapRegistered() {
    static std::optional<bool> last_status;
    bool current_status = false;

    if (node_->get_parameter("map", shared_data_->map_param_)) {
        try {
            nlohmann::json jsonData = nlohmann::json::parse(shared_data_->map_param_);
            int map_id = jsonData[0]["id"];
            std::string map_name = jsonData[0]["name"];

            current_status = true;

            if (last_status != current_status) {
                RCLCPP_INFO(node_->get_logger(), "Mapa atual: name: %s, id: %d", map_name.c_str(), map_id);
            }
        } catch (...) {
            current_status = false;
            if (last_status != current_status) {
                RCLCPP_ERROR(node_->get_logger(), "Erro ao interpretar o JSON do parâmetro 'map'.");
            }
        }
    } else {
        current_status = false;
        if (last_status != current_status) {
            RCLCPP_WARN(node_->get_logger(), "Parâmetro 'map' não encontrado.");
        }
    }

    last_status = current_status;
    return current_status;
}

bool OffState::getSettings() {
    static std::optional<bool> last_status;
    std::string attempts_str;
    bool current_status = false;

    if (node_->get_parameter("setting/dockDetectionAttempts", attempts_str)) {
        try {
            shared_data_->setting_dock_detection_attempts_param = std::stoi(attempts_str);
            current_status = true;
            if (last_status != current_status) {
                RCLCPP_INFO(node_->get_logger(), "dockDetectionAttempts: %d", shared_data_->setting_dock_detection_attempts_param);
            }
        } catch (const std::exception& e) {
            current_status = false;
            if (last_status != current_status) {
                RCLCPP_ERROR(node_->get_logger(), "Erro ao converter 'dockDetectionAttempts': %s", e.what());
            }
        }
    } else {
        current_status = false;
        if (last_status != current_status) {
            RCLCPP_WARN(node_->get_logger(), "Parâmetro 'setting/dockDetectionAttempts' não encontrado.");
        }
    }

    last_status = current_status;
    return current_status;
}

State::StateType OffState::getType() const {
    return State::StateType::OFF;
}

} // namespace State
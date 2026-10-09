#include "states/state.hpp"
#include "decision_maker_node.hpp"
#include <shared_data.hpp>

#include "states/off_state.hpp"
#include "states/idle_state.hpp"
#include "states/navigation_state.hpp"
#include "states/docking_state.hpp"

namespace state {

State::State(std::shared_ptr<rclcpp::Node> node) : node_(node), shared_data_(SharedData::getInstance()) {
    shared_data_ = SharedData::getInstance();
    log_client_ = node_->create_client<drn_integration_msgs::srv::Log>("/drn_integration/communication/log_service");
    alert_client_ = node_->create_client<drn_integration_msgs::srv::Alert>("/drn_integration/communication/alert_service");

}

std::shared_ptr<State> State::createState(StateType type, std::shared_ptr<rclcpp::Node> node) {
    switch (type) {
        case StateType::OFF:
            return std::make_shared<OffState>(node);
        case StateType::IDLE:
            return std::make_shared<IdleState>(node);
        case StateType::NAVIGATION:
            return std::make_shared<NavigationState>(node);
        case StateType::DOCKING:
            return std::make_shared<DockingState>(node);
        default:
            return nullptr;
    }
}

std::shared_ptr<State> State::transition(StateType type) {
    return State::createState(type, node_);
}

std::shared_ptr<State> State::getCurrentState() {
    return next_state_;
}

bool State::sendAlert(const std::string& uuid, bool active, int errorcode, const std::string& data, const std::string& message, const std::string& timestamp){
    
    auto request = std::make_shared<drn_integration_msgs::srv::Alert::Request>();
    
    request->uuid = uuid;
    request->active = active;
    request->errorcode = errorcode;
    request->data = data;
    request->message = message;
    request->timestamp = timestamp;
    
    auto result_future = alert_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        RCLCPP_INFO(node_->get_logger(), "Alert sent: %s, %s", response->success ? "True" : "False", response->feedback.c_str());
        return response->success;
    } else {
        RCLCPP_WARN(node_->get_logger(), "Timeout: '/alert_service' did not respond within 1 second.");
        return false;
    }
}

void State::checkAndHandleAlert(ErrorCode code, bool alert_active, const std::string& message) {
    auto it = this->shared_data_->alert_map.find(code);
    std::string data = error_code_description.at(code);
    std::time_t timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string uuid = std::to_string(timestamp);

    if (alert_active) {
        if (it == this->shared_data_->alert_map.end()) {
            bool feedback = false;
            while (!feedback) {
                feedback = this->sendAlert(uuid, true, static_cast<int>(code), data, message, uuid);
            }
            this->shared_data_->alert_map[code] = uuid;
            RCLCPP_WARN(node_->get_logger(), "%s alert!", data.c_str());
        }
    } else if (it != this->shared_data_->alert_map.end()) {
        std::string existing_uuid = it->second;
        bool feedback = false;
        while (!feedback) {
            feedback = this->sendAlert(existing_uuid, false, static_cast<int>(code), data, message, existing_uuid);
        }
        this->shared_data_->alert_map.erase(it);
        RCLCPP_INFO(node_->get_logger(), "%s alert solved!", data.c_str());
    }
}


bool State::sendLog(const std::string& uuid, const std::string& data, const std::string& timestamp){
    
    auto request = std::make_shared<drn_integration_msgs::srv::Log::Request>();
    
    request->uuid = uuid;
    request->data = data;
    request->timestamp = timestamp;
    
    auto result_future = log_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        RCLCPP_INFO(node_->get_logger(), "Log sent: %s, %s", response->success ? "True" : "False", response->feedback.c_str());
        return response->success;
    } else {
        RCLCPP_WARN(node_->get_logger(), "Timeout: '/log_service' did not respond within 1 second.");
        return false;
    }
}

} // namespace State
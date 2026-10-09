#include "states/docking_state.hpp"
// #include "states/idle_state.hpp"
// #include "states/error_state.hpp"
#include "decision_maker_node.hpp"

namespace state {

DockingState::DockingState(std::shared_ptr<rclcpp::Node> node) 
    : State(node)
{    
    docking_action_client_ = rclcpp_action::create_client<drn_navigation_msgs::action::DockingParam>(node, "/docking_align"); 
}

void DockingState::enter() {
    RCLCPP_INFO(node_->get_logger(), "Entering DOCKING state");
    docking_attempts_ = 0;
    shared_data_->robot_docked = false;
    docking_ = false;
    // behavior_tree_->resetTree();
}

void DockingState::execute() {
    auto decision_maker_node = static_cast<DecisionMakerNode*>(this->node_.get());
    if (docking_attempts_ >= shared_data_->setting_dock_detection_attempts_param) {
        RCLCPP_INFO(node_->get_logger(), "Max docking attempts reached");
        // this->shared_data_->robot_status = false;
        shared_data_->resetMemory();

        auto now = std::chrono::system_clock::now();
        std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
        std::string uuid = std::to_string(timestamp);
        std::stringstream data;
        data << "Missão " << shared_data_->mission_id << " finalizada com falha pois não foi encontrada a doca!";
        this->sendLog(uuid, data.str(), std::to_string(timestamp));

        decision_maker_node->sendMissionLog(shared_data_->mission_id, "FINALIZADA", "Missão finalizada com falha pois não foi encontrada a doca!");
        decision_maker_node->sendMissionResponse(mission_response::finishMission, false, shared_data_->mission_id);
        next_state_ = transition(StateType::IDLE);
        return;
    }

    if (shared_data_->robot_docked) {
        shared_data_->resetMemory();
        decision_maker_node->sendMissionLog(shared_data_->mission_id, "FINALIZADA", "Missão finalizada com sucesso!");
        decision_maker_node->sendMissionResponse(mission_response::finishMission, true, shared_data_->mission_id);
        next_state_ = transition(StateType::IDLE);
        return;
    }
    else if (!docking_) {
        docking_ = true;
        if (!docking_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->node_->get_logger(), "Servidor de ação de docking não disponível.");
            return;
        }
        auto goal_msg = drn_navigation_msgs::action::DockingParam::Goal();
        goal_msg.distance_seg = 1.5;
        goal_msg.linear_tolerance = 0.01;
        goal_msg.angular_tolerance = 0.03;
        goal_msg.direction = shared_data_->mission_direction;

        auto send_goal_options = rclcpp_action::Client<drn_navigation_msgs::action::DockingParam>::SendGoalOptions();

        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<drn_navigation_msgs::action::DockingParam>::WrappedResult &result) {
            if (result.result->success){
                RCLCPP_INFO(this->node_->get_logger(), "Primeira meta concluída. Enviando segunda meta (1.0)...");
    
                auto second_goal_msg = drn_navigation_msgs::action::DockingParam::Goal();
                second_goal_msg.distance_seg = 1.0;
                second_goal_msg.linear_tolerance = 0.01;
                second_goal_msg.angular_tolerance = 0.03;
                second_goal_msg.direction = shared_data_->mission_direction;
    
                auto second_goal_options = rclcpp_action::Client<drn_navigation_msgs::action::DockingParam>::SendGoalOptions();
    
                second_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<drn_navigation_msgs::action::DockingParam>::WrappedResult &second_result) {
                    if (second_result.result->success){
                        RCLCPP_INFO(this->node_->get_logger(), "Segunda meta concluída. Enviando terceira meta (0.5)...");
    
                        auto third_goal_msg = drn_navigation_msgs::action::DockingParam::Goal();
                        third_goal_msg.distance_seg = 0.5;
                        third_goal_msg.linear_tolerance = 0.01;
                        third_goal_msg.angular_tolerance = 0.03;
                        third_goal_msg.direction = shared_data_->mission_direction;
    
                        auto third_goal_options = rclcpp_action::Client<drn_navigation_msgs::action::DockingParam>::SendGoalOptions();
    
                        third_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<drn_navigation_msgs::action::DockingParam>::WrappedResult &third_result) {
                            if (third_result.result->success){
                                shared_data_->robot_docked = true;
                                RCLCPP_INFO(this->node_->get_logger(), "Terceira meta concluída com sucesso! Docking finalizado.");
                            } else {
                                docking_ = false;
                                RCLCPP_WARN(this->node_->get_logger(), "Terceira meta falhou.");
                            }
                        };
    
                        docking_action_client_->async_send_goal(third_goal_msg, third_goal_options);
    
                    } else {
                        docking_ = false;
                        RCLCPP_WARN(this->node_->get_logger(), "Segunda meta falhou.");
                    }
                };
    
                docking_action_client_->async_send_goal(second_goal_msg, second_goal_options);
    
            } else {
                docking_ = false;
                RCLCPP_WARN(this->node_->get_logger(), "Primeira meta falhou.");
            }
        };

        docking_action_client_->async_send_goal(goal_msg, send_goal_options);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    docking_attempts_ += 1;
    }
}

void DockingState::exit() {
    RCLCPP_INFO(node_->get_logger(), "Exiting DOCKING state");
}

State::StateType DockingState::getType() const {
    return State::StateType::DOCKING;
}

}// namespace State
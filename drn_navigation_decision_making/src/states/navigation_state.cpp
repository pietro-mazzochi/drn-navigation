#include "states/navigation_state.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
// #include "states/docking_state.hpp"
// #include "states/waiting_state.hpp"
// #include "states/idle_state.hpp"
#include "decision_maker_node.hpp"

namespace state {

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

NavigationState::NavigationState(std::shared_ptr<rclcpp::Node> node)
    : State(node)
{
    route_planning_client_ = node_->create_client<drn_navigation_msgs::srv::GraphPath>("/drn_navigation/route_planning/calculate_global_steps_service");
    navigate_through_poses_client_  = rclcpp_action::create_client<NavigateThroughPoses>(node_, "/navigate_through_poses");
}

void NavigationState::enter() {
    RCLCPP_INFO(node_->get_logger(), "Entering NAVIGATION state");
    navigation_start_time_ = node_->now();
    shared_data_->navigate_complete_mission = false;
    }

void NavigationState::execute() {
    // RCLCPP_INFO(node_->get_logger(), "Executing NAVIGATION state");

    if (shared_data_->mission_canceled){
        next_state_ = transition(StateType::IDLE);
        shared_data_->mission_canceled = false;
        return;
    }

    if ((this->shared_data_->new_mission || route_replanning_) && !this->shared_data_->robot_pause){
        route_replanning_ = false;
        auto request = std::make_shared<drn_navigation_msgs::srv::GraphPath::Request>();
        request->id_target = shared_data_->mission_end_id;
        request->robot_pose.header.frame_id = "map";
        request->robot_pose.header.stamp = node_->now();  // Timestamp atual do nó

        RCLCPP_INFO(this->node_->get_logger(), "Pose atual: %f, %f", shared_data_->robot_pose.pose.position.x, shared_data_->robot_pose.pose.position.y);
        // Configurar a posição
        request->robot_pose = shared_data_->robot_pose;
        shared_data_->initial_pose_mission = shared_data_->robot_pose;
        request->blocked_paths = shared_data_->blocked_paths;
        request->direction = shared_data_->mission_direction;

        auto result_future = route_planning_client_->async_send_request(request);
        auto goal_msg = NavigateThroughPoses::Goal();

        if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = result_future.get();
            try{
                path = response->path;
                edges_ = response->relative_edges;
                shared_data_->total_mission_poses = response->path.size();
                goal_msg.poses = response->path;
                // goal_msg.poses.push_back(response->path)
                if(response->path.size() == 0){
                    shared_data_->new_mission = false;
                    RCLCPP_INFO(this->node_->get_logger(), "Empty path!");
                    route_replanning_ = false;
                    shared_data_->resetMemory();    
                    auto decision_maker_node = static_cast<DecisionMakerNode*>(this->node_.get());
                    decision_maker_node->sendMissionLog(shared_data_->mission_id, "FINALIZADA", "Missão finalizada com falha pois Path vazio");
                    decision_maker_node->sendMissionResponse(mission_response::finishMission, false, shared_data_->mission_id);
                    next_state_ = transition(StateType::IDLE);
                    return;
                }
                RCLCPP_INFO(node_->get_logger(), "Path obtido com %zu poses, dock: %s", response->path.size(), response->dock_robot ? "true" : "false");
            }
            catch (...){ 
                RCLCPP_INFO(node_->get_logger(), "Path não obtido");
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "Timeout: '/drn_navigation/route_planning/calculate_global_steps_service' did not respond within 1 second.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Iniciando missão para o nodo %d", shared_data_->mission_end_id);

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();

        send_goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleNavigateThroughPoses> goal_handle) {
            // std::ostringstream oss;
            // for (size_t i = 0; i < edges_.size(); ++i) {
            //     const auto& pose = edges_[i];
            //     oss << "\n\t[" << pose.id_source
            //         << "," << pose.id_target
            //         << "]";
            // }
            // RCLCPP_INFO(this->node_->get_logger(), "Path: [%s]", oss.str().c_str());
            if (!goal_handle) {
                RCLCPP_ERROR(this->node_->get_logger(), "Goal was rejected by the server");
            } else {
                RCLCPP_INFO(this->node_->get_logger(), "Goal accepted by the server, waiting for result");
            }
        };
        send_goal_options.feedback_callback = [this](GoalHandleNavigateThroughPoses::SharedPtr,
            const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
            if (!feedback) {
                RCLCPP_WARN(node_->get_logger(), "Recebeu feedback vazio.");
                return;
            }
            shared_data_->number_of_poses_remaining = feedback->number_of_poses_remaining;
            shared_data_->current_edge = edges_[edges_.size() - feedback->number_of_poses_remaining];
            // RCLCPP_INFO(node_->get_logger(), "Recebido feedback! Poses restantes: %d de %d totais", feedback->number_of_poses_remaining, shared_data_->total_mission_poses);
        };

        send_goal_options.result_callback = [this](const GoalHandleNavigateThroughPoses::WrappedResult &result) {
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                shared_data_->navigate_complete_mission = true;
                if(shared_data_->mission_type == mission_type::transfer){
                    shared_data_->resetMemory();
                    auto decision_maker_node = static_cast<DecisionMakerNode*>(this->node_.get());
                    decision_maker_node->sendMissionLog(shared_data_->mission_id, "FINALIZADA", "Missão finalizada com sucesso!");
                    decision_maker_node->sendMissionResponse(mission_response::finishMission, true, shared_data_->mission_id);
                    next_state_ = transition(StateType::IDLE);
                }
                if(shared_data_->mission_type == mission_type::loading || shared_data_->mission_type == mission_type::unloading){
                    next_state_ = transition(StateType::DOCKING);
                }
                RCLCPP_INFO(this->node_->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->node_->get_logger(), "Goal was aborted in pose: %f, %f", shared_data_->robot_pose.pose.position.x, shared_data_->robot_pose.pose.position.y);
                RCLCPP_INFO(this->node_->get_logger(), "Route replanning, blocked edge = %d, (%ld, %ld)!", (shared_data_->total_mission_poses - shared_data_->number_of_poses_remaining), shared_data_->current_edge.id_source, shared_data_->current_edge.id_target);
                if((shared_data_->current_edge.id_source > 0) && (shared_data_->current_edge.id_source != shared_data_->current_edge.id_target) && (shared_data_->number_of_poses_remaining > 0)){
                    shared_data_->blocked_paths.push_back(shared_data_->current_edge);
                    route_replanning_ = true;
                }
                else {
                    shared_data_->resetMemory();    
                    auto decision_maker_node = static_cast<DecisionMakerNode*>(this->node_.get());
                    decision_maker_node->sendMissionLog(shared_data_->mission_id, "FINALIZADA", "Missão finalizada com falha, pois action foi abortado");
                    decision_maker_node->sendMissionResponse(mission_response::finishMission, false, shared_data_->mission_id);
                    next_state_ = transition(StateType::IDLE);
                }
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->node_->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->node_->get_logger(), "Unknown result code");
                break;
            }
        };
        auto nav_result_future = navigate_through_poses_client_->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(node_, nav_result_future, std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS) {
            return;
        }
        shared_data_->new_mission = false;
    }

    double dx = shared_data_->initial_pose_mission.pose.position.x - shared_data_->robot_pose.pose.position.x;
    double dy = shared_data_->initial_pose_mission.pose.position.y - shared_data_->robot_pose.pose.position.y;
    if(std::sqrt(dx * dx + dy * dy) >= shared_data_->ls_config_distance){
        shared_data_->ls_config.data = 0x95;
    }
}

void NavigationState::exit() {
    RCLCPP_INFO(node_->get_logger(), "Exiting NAVIGATION state");
}

State::StateType NavigationState::getType() const {
    return State::StateType::NAVIGATION;
}

} // namespace State
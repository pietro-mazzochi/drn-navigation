#include "states/idle_state.hpp"
// #include "states/navigation_state.hpp"
// #include "states/docking_state.hpp"

namespace state {

IdleState::IdleState(std::shared_ptr<rclcpp::Node> node) 
    : State(node)
{

}

void IdleState::enter() {
    RCLCPP_INFO(node_->get_logger(), "Entering IDLE state");
}

void IdleState::execute() {
    // RCLCPP_INFO(node_->get_logger(), "Executing IDLE state");
    if (this->shared_data_->new_mission && !this->shared_data_->robot_pause) {
        RCLCPP_INFO(node_->get_logger(), "New mission received, transitioning to NAVIGATION");
        next_state_ = transition(StateType::NAVIGATION);
        return;
    }
    // next_state_ = transition(StateType::IDLE);
}

void IdleState::exit() {
    RCLCPP_INFO(node_->get_logger(), "Exiting IDLE state");
}

State::StateType IdleState::getType() const {
    return State::StateType::IDLE;
}

} // namespace State
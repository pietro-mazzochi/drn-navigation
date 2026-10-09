#pragma once
#include "states/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <drn_navigation_msgs/action/docking_param.hpp>


class DecisionMakerNode;

namespace state {

class DockingState : public State {
public:
    explicit DockingState(std::shared_ptr<rclcpp::Node> node);
    void enter() override;
    void execute() override;
    void exit() override;
    StateType getType() const override;
    
private:
    rclcpp_action::Client<drn_navigation_msgs::action::DockingParam>::SharedPtr docking_action_client_;

    int docking_attempts_{0};
    bool docking_{false};

};

} // namespace State
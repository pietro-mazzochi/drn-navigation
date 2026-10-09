#pragma once
#include "states/state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace state {

class IdleState : public State {
public:
    explicit IdleState(std::shared_ptr<rclcpp::Node> node);
    void enter() override;
    void execute() override;
    void exit() override;
    StateType getType() const override;
};

} // namespace State
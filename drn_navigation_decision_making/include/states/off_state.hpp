#pragma once
#include "states/state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <nlohmann/json.hpp>

namespace state {

class OffState : public State {
public:
    explicit OffState(std::shared_ptr<rclcpp::Node> node);
    void enter() override;
    void execute() override;
    void exit() override;
    StateType getType() const override;
private:
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
    int max_docking_attempts_{0};
    bool getSettings();
    bool isRobotRegistered();
    bool isMapRegistered();
};

} // namespace State
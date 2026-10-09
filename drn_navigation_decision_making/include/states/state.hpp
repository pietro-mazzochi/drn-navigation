#pragma once
#include <memory>
#include <cmath>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <shared_data.hpp>
#include <drn_integration_msgs/srv/alert.hpp>
#include <drn_integration_msgs/srv/log.hpp>

namespace state{
class State {
public:
    enum class StateType {
        OFF,
        IDLE,
        NAVIGATION,
        DOCKING
    };

    explicit State(std::shared_ptr<rclcpp::Node> node);
    virtual ~State() = default;

    virtual void enter() = 0;
    virtual void execute() = 0;
    virtual void exit() = 0;
    virtual StateType getType() const = 0;
    virtual std::shared_ptr<State> getCurrentState();

    static std::shared_ptr<State> createState(StateType type, std::shared_ptr<rclcpp::Node> node);

    bool sendAlert(const std::string& uuid, bool active, int errorcode, const std::string& data, const std::string& message, const std::string& timestamp);
    bool sendLog(const std::string& uuid, const std::string& data, const std::string& timestamp);
    void checkAndHandleAlert(ErrorCode code, bool condition_failed, const std::string& message);

protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<State> next_state_;
    std::shared_ptr<SharedData> shared_data_;
    std::shared_ptr<State> transition(StateType type);
    rclcpp::Client<drn_integration_msgs::srv::Alert>::SharedPtr alert_client_;
    rclcpp::Client<drn_integration_msgs::srv::Log>::SharedPtr log_client_;
};

} // namespace State
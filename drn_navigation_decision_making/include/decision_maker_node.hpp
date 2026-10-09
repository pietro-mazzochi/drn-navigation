// decision_maker_node.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "shared_data.hpp"
#include "states/state.hpp"
#include "drn_integration_msgs/msg/mission.hpp" 
#include "drn_navigation_msgs/msg/robot_status.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <drn_integration_msgs/srv/mission_log.hpp>
#include "drn_integration_msgs/srv/mission.hpp" 
#include <drn_integration_msgs/srv/alert.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include "siemens_et200sp_modbus_driver_msgs/msg/siemens_etsp_modbus_driver.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
// #include "frequency_check_msg/msg/frequency_check.hpp" 

class DecisionMakerNode : public rclcpp::Node {
public:
    explicit DecisionMakerNode();
    void initialize();
    void run();
    void sendMissionLog(int id, const std::string& status, const std::string& log_message);
    void sendMissionResponse(const std::string& command, bool status, int id);
    void sendAlert(const std::string& uuid, bool active, int errorcode, const std::string& data, const std::string& message, const std::string& timestamp);
    private:
    // Shared data instance
    std::shared_ptr<SharedData> shared_data_;
    
    // Current state
    std::shared_ptr<state::State> current_state_;
    
    // ROS 2 communications
    rclcpp::Subscription<drn_integration_msgs::msg::Mission>::SharedPtr mission_sub_;
    // rclcpp::Publisher<drn_navigation_msgs::msg::RobotStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ls_config_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_mission_client_;
    // rclcpp::Subscription<frequency_check_msg::msg::FrequencyCheck>::SharedPtr sensors_sub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters);

    
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped >::SharedPtr amcl_pose_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped >::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>::SharedPtr plc_msg_sub_;


    // Callback functions
    void missionCallback(const drn_integration_msgs::msg::Mission::SharedPtr msg);
    void publishStatus();
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void modeCallback(const siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver::SharedPtr msg);

    // void sensorErrorCallback(const frequency_check_msg::msg::FrequencyCheck::SharedPtr msg);
    void cancelMissionService();

    // Mission Log
    rclcpp::Client<drn_integration_msgs::srv::MissionLog>::SharedPtr mission_log_client_;
    rclcpp::Client<drn_integration_msgs::srv::Alert>::SharedPtr alert_client_;
    rclcpp::Client<drn_integration_msgs::srv::Mission>::SharedPtr mission_response_client_;
};
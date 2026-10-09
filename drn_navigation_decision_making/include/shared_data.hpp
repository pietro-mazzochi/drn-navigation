#pragma once
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "drn_navigation_msgs/msg/edge.hpp"
#include "std_msgs/msg/int32.hpp"

enum class ErrorCode {
    DOCK_NOT_FOUND = 10,
    ROUTE_NOT_FOUND = 20,
    UNCONFIGURED_MAP = 30,
    UNREGISTERED_ROBOT = 40,
    INCONSISTENT_SENSOR = 50
};

const std::unordered_map<ErrorCode, std::string> error_code_description = {
    {ErrorCode::DOCK_NOT_FOUND, "Doca não encontrada"},
    {ErrorCode::ROUTE_NOT_FOUND, "Rota inexistente"},
    {ErrorCode::UNCONFIGURED_MAP, "Mapa não configurado"},
    {ErrorCode::UNREGISTERED_ROBOT, "Robô não configurado"},
    {ErrorCode::INCONSISTENT_SENSOR, "Leitura inconsistente de sensor"}
};

namespace mission_response {
    const std::string startMission = "0xCA";
    const std::string pauseMission = "0xD2";
    const std::string continueMission = "0xD1";
    const std::string cancelMission = "0xCB";
    const std::string finishMission = "0xD5";
}

namespace mission_type {
    const std::string transfer = "transfer";
    const std::string loading = "loading";
    const std::string unloading = "unloading";
}

class SharedData {
public:
    static std::shared_ptr<SharedData> getInstance() {
        static std::shared_ptr<SharedData> instance(new SharedData());
        return instance;
    }

    void resetMemory() {
        active_mission = false;
        new_mission = false;
        robot_pause = false;
        mission_type = "";
        total_mission_poses = 0;
        mission_direction = "front";
        current_edge.id_source = 0;
        current_edge.id_target = 0;
        number_of_poses_remaining = 0;
        robot_docked = false;
        blocked_paths.clear();
    }

    bool active_mission{false};
    bool new_mission{false};
    bool robot_pause{false};
    bool robot_status{false};
    bool robot_docked{false};
    bool robot_auto_mode{false};
    bool mission_canceled{false};
    int mission_id{0};
    int mission_end_id{0};
    float battery_level{100.0};
    bool is_charging{false};
    bool obstacle_detected{false};
    bool navigate_complete_mission{false};
    int max_docking_attempts{3};
    int total_mission_poses{0};
    int number_of_poses_remaining{0};
    double displacement_seg{0.38};
    drn_navigation_msgs::msg::Edge current_edge; 
    std::string mission_type{};
    geometry_msgs::msg::PoseStamped robot_pose;
    geometry_msgs::msg::PoseStamped initial_pose_mission;
    std::unordered_map<ErrorCode, std::string> alert_map;
    std::vector<drn_navigation_msgs::msg::Edge> blocked_paths;
    std_msgs::msg::Int32 ls_config;
    double ls_config_distance{1};
    std::string mission_direction{"front"};
    std::string vehicle_param_, map_param_;
    int setting_dock_detection_attempts_param;

private:
    SharedData(){
        ls_config.data = 0x95;
    };
};
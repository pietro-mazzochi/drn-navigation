#pragma once
#include "states/state.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <drn_navigation_msgs/srv/graph_path.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/client.hpp>

class DecisionMakerNode;

namespace state {

using PoseStamped = std::vector<geometry_msgs::msg::PoseStamped>;

class NavigationState : public State {
public:
    explicit NavigationState(std::shared_ptr<rclcpp::Node> node);
    void enter() override;
    void execute() override;
    void exit() override;
    StateType getType() const override;
    PoseStamped path;
    std::vector<drn_navigation_msgs::msg::Edge> edges_;

private:
    rclcpp::Time navigation_start_time_;
    bool route_replanning_{false};

    rclcpp::Client<drn_navigation_msgs::srv::GraphPath>::SharedPtr route_planning_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
};

} // namespace state
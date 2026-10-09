#include <string>
#include <memory>
#include <cmath>

//#include "nav2_behavior_tree/plugins/action/wait_action.hpp"
#include "custom_bt/wait_print.hpp"

namespace nav2_behavior_tree
{

WaitPrint::WaitPrint(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

void WaitPrint::initialize()
{
  double duration;
  getInput("wait_duration", duration);
  if (duration <= 0) {
    RCLCPP_WARN(
      node_->get_logger(), "Wait duration is negative or zero "
      "(%f). Setting to positive.", duration);
    duration *= -1;
  }

  goal_.time = rclcpp::Duration::from_seconds(duration);
  RCLCPP_WARN(node_->get_logger(), "Inicialiazei o wait com %f",duration);
}

void WaitPrint::on_tick()
{
    // if (!BT::isStatusActive(status())) {
    if(status() != BT::NodeStatus::IDLE){
        // RCLCPP_WARN(node_->get_logger(), "Inicialiazei o wait com %f",duration);
        initialize();
    }
    increment_recovery_count();
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::WaitPrint>(name, "wait", config);
    };

  factory.registerBuilder<nav2_behavior_tree::WaitPrint>("WaitPrint", builder);
}
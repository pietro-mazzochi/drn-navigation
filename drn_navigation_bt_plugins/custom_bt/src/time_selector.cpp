#include <string>
#include <memory>

#include "std_msgs/msg/float64.hpp"

#include "custom_bt/time_selector.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

TimeSelector::TimeSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf), last_selected_time_(0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("topic_name", topic_name_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  time_selector_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    topic_name_,
    qos,
    std::bind(&TimeSelector::callbackTimeSelect, this, _1),
    sub_option);
}

BT::NodeStatus TimeSelector::tick()
{
  callback_group_executor_.spin_some();

  if (last_selected_time_== 0) {
    double default_time;
    
    if (!getInput("default_time", default_time)) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_time_ = default_time;
    }
  }

  setOutput("selected_time", last_selected_time_);

  return BT::NodeStatus::SUCCESS;
}

void
TimeSelector::callbackTimeSelect(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_selected_time_ = msg->data;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Último tempo selecionado: %f", last_selected_time_);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeSelector>("TimeSelector");
}
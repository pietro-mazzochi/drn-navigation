#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief The TimeSelector behavior is used to switch the controller
 * that will be used by the controller server. It subscribes to a topic "time_selector"
 * to get the decision about time to wait. The time_controller output port is passed 
 * to plugin wait input port.
 */
class TimeSelector : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::TimeSelector
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  TimeSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "default_time",
        "the default time to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name",
        "time_selector",
        "the input topic name to select the time"),

      BT::OutputPort<double>(
        "selected_time",
        "Selected time by subscription")
    };
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the controller_selector topic
   *
   * @param msg the message with the id of the controller_selector
   */
  void callbackTimeSelect(const std_msgs::msg::Float64::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr time_selector_sub_;

  double last_selected_time_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_SELECTOR_NODE_HPP_
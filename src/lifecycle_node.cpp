//
// Created by guanlin on 25-2-13.
//

#include <rm_ros2_manual/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

namespace rm_ros2_manual
{
LifecycleNode::LifecycleNode(const std::string& node_name) : LifecycleNode(node_name)
{
  RCLCPP_INFO(get_logger(), "Lifecycle node base constructor called.");
  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

CallbackReturn LifecycleNode::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "LifecycleNodeBase on_configure() is called.");
  std::string robot;
  this->get_parameter_or("robot_type", robot, static_cast<std::string>("error"));
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleNode::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "LifecycleNodeBase on_activate() is called.");
  loop_running_ = true;
  loop_thread_ = std::thread([this]() {
    while (loop_running_ && rclcpp::ok())
    {
      run();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "LifecycleNodeBase on_deactivate() is called.");
  loop_running_ = false;
  if (loop_thread_.joinable())
    loop_thread_.join();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "LifecycleNodeBase on_cleanup() is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "LifecycleNodeBase on_shutdown() is called.");
  return CallbackReturn::SUCCESS;
}
}  // namespace rm_ros2_manual
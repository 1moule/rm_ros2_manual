//
// Created by guanlin on 25-2-12.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace rm_ros2_manual
{
class ManualBase : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ManualBase(const std::string& node_name);
  CallbackReturn on_configure(const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*state*/) override;

protected:
  virtual void run()
  {
    RCLCPP_INFO(this->get_logger(), "Running main loop");
  }

private:
  std::thread loop_thread_;
  bool loop_running_ = false;
};
}  // namespace rm_ros2_manual
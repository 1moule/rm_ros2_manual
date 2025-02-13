//
// Created by guanlin on 25-2-12.
//

#pragma once

#include "rm_ros2_manual/manual_base.hpp"

#include <thread>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace rm_ros2_manual
{
class ManualNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ManualNode(const std::string& node_name);
  CallbackReturn on_configure(const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*state*/) override;

private:
  std::thread loop_thread_;
  std::shared_ptr<ManualBase> manual_control_;
  bool loop_running_ = false;
};
}  // namespace rm_ros2_manual
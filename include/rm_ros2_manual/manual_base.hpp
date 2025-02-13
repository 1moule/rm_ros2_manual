//
// Created by guanlin on 25-2-13.
//

#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace rm_ros2_manual
{
class ManualBase
{
public:
  explicit ManualBase(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  void run();

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
}  // namespace rm_ros2_manual

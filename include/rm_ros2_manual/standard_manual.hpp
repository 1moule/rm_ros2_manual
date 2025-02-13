//
// Created by guanlin on 25-2-13.
//

#pragma once

#include <rm_ros2_manual/manual_base.hpp>
#include <rm_ros2_manual/module/chassis.hpp>

namespace rm_ros2_manual
{
class StandardManual final : public ManualBase
{
public:
  explicit StandardManual(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
};
}  // namespace rm_ros2_manual

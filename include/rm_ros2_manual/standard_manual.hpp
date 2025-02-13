//
// Created by guanlin on 25-2-13.
//

#pragma once

#include <rm_ros2_manual/manual_base.hpp>

namespace rm_ros2_manual
{
class StandardManual final : public ManualBase
{
public:
  explicit StandardManual(const rclcpp::Node::SharedPtr& node);
};
}  // namespace rm_ros2_manual

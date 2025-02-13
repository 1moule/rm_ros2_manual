//
// Created by guanlin on 25-2-13.
//

#include "rm_ros2_manual/manual_base.hpp"

namespace rm_ros2_manual
{
ManualBase::ManualBase(rclcpp_lifecycle::LifecycleNode::SharedPtr node) : node_(node)
{
}

void ManualBase::run()
{
  RCLCPP_INFO(node_->get_logger(), "Running main loop!!");
}

}  // namespace rm_ros2_manual
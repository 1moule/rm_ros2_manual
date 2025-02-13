//
// Created by guanlin on 25-2-13.
//

#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rm_ros2_msgs/msg/dbus_data.hpp>

namespace rm_ros2_manual
{
class ManualBase
{
public:
  virtual ~ManualBase() = default;
  explicit ManualBase(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  virtual void run();

protected:
  virtual void registerPubAndSub();
  // virtual void sendCommand(const rclcpp::Time& time);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<rm_ros2_msgs::msg::DbusData>::SharedPtr dbus_sub_;
};
}  // namespace rm_ros2_manual

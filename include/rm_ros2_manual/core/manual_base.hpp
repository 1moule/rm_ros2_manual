//
// Created by guanlin on 25-2-13.
//

#pragma once

#include "rm_ros2_manual/core/input_event.hpp"
#include <rm_ros2_msgs/msg/dbus_data.hpp>

namespace rm_ros2_manual
{
class ManualBase
{
public:
  explicit ManualBase(const rclcpp::Node::SharedPtr& node);
  virtual ~ManualBase() = default;
  virtual void run();

protected:
  virtual void registerPubAndSub();
  // virtual void sendCommand(const rclcpp::Time& time);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<rm_ros2_msgs::msg::DbusData>::SharedPtr dbus_sub_;
};
}  // namespace rm_ros2_manual

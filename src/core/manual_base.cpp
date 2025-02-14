//
// Created by guanlin on 25-2-13.
//

#include "rm_ros2_manual/core/manual_base.hpp"
#include <rm_ros2_common/tools/ros_tools.hpp>

namespace rm_ros2_manual
{
ManualBase::ManualBase(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  ManualBase::registerPubAndSub();
}

void ManualBase::registerPubAndSub()
{
  const std::string dbus_topic_ = getParam(std::move(node_), "dbus_topic", std::string("/rm_ecat_hw/dbus"));
  auto dbusCallback = [this](const std::shared_ptr<rm_ros2_msgs::msg::DbusData> /*msg*/) -> void {
    RCLCPP_INFO(node_->get_logger(), "Received dbus message");
  };
  dbus_sub_ =
      node_->create_subscription<rm_ros2_msgs::msg::DbusData>(dbus_topic_, rclcpp::SystemDefaultsQoS(), dbusCallback);
}

void ManualBase::run()
{
  RCLCPP_INFO(node_->get_logger(), "Running main loop!!");
}

}  // namespace rm_ros2_manual
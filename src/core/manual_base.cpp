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
  right_switch_down_event_.setRising(std::bind(&ManualBase::rightSwitchDownRise, this));
  right_switch_mid_event_.setRising(std::bind(&ManualBase::rightSwitchMidRise, this));
  right_switch_up_event_.setRising(std::bind(&ManualBase::rightSwitchUpRise, this));
  right_switch_down_event_.setActiveHigh(std::bind(&ManualBase::rightSwitchDownOn, this));
  right_switch_mid_event_.setActiveHigh(std::bind(&ManualBase::rightSwitchMidOn, this));
  right_switch_up_event_.setActiveHigh(std::bind(&ManualBase::rightSwitchUpOn, this));
  left_switch_down_event_.setRising(std::bind(&ManualBase::leftSwitchDownRise, this));
  left_switch_up_event_.setRising(std::bind(&ManualBase::leftSwitchUpRise, this));
  left_switch_mid_event_.setEdge(std::bind(&ManualBase::leftSwitchMidRise, this),
                                 std::bind(&ManualBase::leftSwitchMidFall, this));
}

void ManualBase::registerPubAndSub()
{
  std::string dbus_topic_;
  getParam(node_, "dbus_topic", dbus_topic_, std::string("/rm_ecat_hw/dbus"));
  dbus_sub_ = node_->create_subscription<rm_ros2_msgs::msg::DbusData>(
      dbus_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&ManualBase::dbusDataCallback, this, std::placeholders::_1));
}

void ManualBase::updateRc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data)
{
  left_switch_down_event_.update(dbus_data->s_l == rm_ros2_msgs::msg::DbusData::DOWN);
  left_switch_mid_event_.update(dbus_data->s_l == rm_ros2_msgs::msg::DbusData::MID);
  left_switch_up_event_.update(dbus_data->s_l == rm_ros2_msgs::msg::DbusData::UP);
}

void ManualBase::updatePc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data)
{
  checkKeyboard(dbus_data);
}

void ManualBase::run()
{
  // RCLCPP_INFO(node_->get_logger(), "Running main loop!!");
}

void ManualBase::dbusDataCallback(const rm_ros2_msgs::msg::DbusData::SharedPtr data)
{
  if (node_->get_clock()->now() - data->stamp < rclcpp::Duration::from_seconds(1.0))
  {
    if (!remote_is_open_)
    {
      RCLCPP_INFO(node_->get_logger(), "Remote controller ON");
      remoteControlTurnOn();
      remote_is_open_ = true;
    }
    right_switch_down_event_.update(data->s_r == rm_ros2_msgs::msg::DbusData::DOWN);
    right_switch_mid_event_.update(data->s_r == rm_ros2_msgs::msg::DbusData::MID);
    right_switch_up_event_.update(data->s_r == rm_ros2_msgs::msg::DbusData::UP);

    if (state_ == RC)
      updateRc(data);
    else if (state_ == PC)
      updatePc(data);
  }
  else
  {
    if (remote_is_open_)
    {
      RCLCPP_INFO(node_->get_logger(), "Remote controller OFF");
      remoteControlTurnOff();
      remote_is_open_ = false;
    }
  }

  sendCommand(data->stamp);
}

}  // namespace rm_ros2_manual
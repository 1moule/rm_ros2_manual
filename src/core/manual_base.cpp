//
// Created by guanlin on 25-2-13.
//

#include "rm_ros2_manual/core/manual_base.hpp"
#include <rm_ros2_common/tools/ros_tools.hpp>
#include <utility>

namespace rm_ros2_manual
{
ManualBase::ManualBase(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  ManualBase::registerPubAndSub();
  referee_last_get_stamp_ = node_->get_clock()->now();
  controller_manager_ = std::make_shared<rm_ros2_common::ControllerManager>(node_);
  right_switch_down_event_.setRising([this] { rightSwitchDownRise(); });
  right_switch_mid_event_.setRising([this] { rightSwitchMidRise(); });
  right_switch_up_event_.setRising([this] { rightSwitchUpRise(); });
  right_switch_down_event_.setActiveHigh([this] { rightSwitchDownOn(); });
  right_switch_mid_event_.setActiveHigh([this] { rightSwitchMidOn(); });
  right_switch_up_event_.setActiveHigh([this] { rightSwitchUpOn(); });
  left_switch_down_event_.setRising([this] { leftSwitchDownRise(); });
  left_switch_up_event_.setRising([this] { leftSwitchUpRise(); });
  left_switch_mid_event_.setEdge([this] { leftSwitchMidRise(); }, [this] { leftSwitchMidFall(); });
}

void ManualBase::registerPubAndSub()
{
  // Dbus
  const std::string dbus_topic_ = getParam(node_, "dbus_topic", std::string("/rm_ecat_hw/dbus"));
  dbus_sub_ = node_->create_subscription<rm_ros2_msgs::msg::DbusData>(
      dbus_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&ManualBase::dbusDataCallback, this, std::placeholders::_1));
  // Referee
  game_status_sub_ = node_->create_subscription<rm_ros2_msgs::msg::GameStatus>(
      "game_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&ManualBase::gameStatusCallback, this, std::placeholders::_1));
  game_robot_hp_sub_ = node_->create_subscription<rm_ros2_msgs::msg::GameRobotHp>(
      "game_robot_hp", rclcpp::SystemDefaultsQoS(),
      std::bind(&ManualBase::gameRobotHpCallback, this, std::placeholders::_1));
  game_robot_status_sub_ = node_->create_subscription<rm_ros2_msgs::msg::GameRobotStatus>(
      "game_robot_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&ManualBase::gameRobotStatusCallback, this, std::placeholders::_1));
  power_heat_data_sub_ = node_->create_subscription<rm_ros2_msgs::msg::PowerHeatData>(
      "power_heat_data", rclcpp::SystemDefaultsQoS(),
      std::bind(&ManualBase::powerHeatDataCallback, this, std::placeholders::_1));
  capacity_sub_ = node_->create_subscription<rm_ros2_msgs::msg::PowerManagementSampleAndStatusData>(
      "capacity", rclcpp::SystemDefaultsQoS(),
      std::bind(&ManualBase::capacityDataCallback, this, std::placeholders::_1));
}

void ManualBase::checkReferee()
{
  chassis_power_on_event_.update(chassis_output_on_);
  gimbal_power_on_event_.update(gimbal_output_on_);
  shooter_power_on_event_.update(shooter_output_on_);
  referee_is_online_ = (node_->get_clock()->now() - referee_last_get_stamp_ < rclcpp::Duration::from_seconds(0.3));
  // manual_to_referee_pub_.publish(manual_to_referee_pub_data_);
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

void ManualBase::robotDie()
{
  if (remote_is_open_)
    controller_manager_->deactivateMainControllers();
}

void ManualBase::robotRevive()
{
  if (remote_is_open_)
    controller_manager_->activateMainControllers();
}

void ManualBase::remoteControlTurnOff()
{
  controller_manager_->deactivateMainControllers();
  state_ = PASSIVE;
}

void ManualBase::remoteControlTurnOn()
{
  controller_manager_->activateMainControllers();
  state_ = IDLE;
}

void ManualBase::run()
{
  checkReferee();
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

void ManualBase::gameRobotStatusCallback(const rm_ros2_msgs::msg::GameRobotStatus::SharedPtr data)
{
  robot_id_ = data->robot_id;
  chassis_output_on_ = data->mains_power_chassis_output;
  gimbal_output_on_ = data->mains_power_gimbal_output;
  shooter_output_on_ = data->mains_power_shooter_output;
  robot_hp_event_.update(data->remain_hp != 0);
}

void ManualBase::powerHeatDataCallback(const rm_ros2_msgs::msg::PowerHeatData::SharedPtr data)
{
  chassis_power_ = data->chassis_power;
  referee_last_get_stamp_ = data->stamp;
}

}  // namespace rm_ros2_manual
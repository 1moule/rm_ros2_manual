//
// Created by guanlin on 25-2-13.
//

#pragma once

#include <rm_ros2_common/decision/controller_manager.hpp>

#include <rm_ros2_msgs/msg/dbus_data.hpp>
#include <rm_ros2_msgs/msg/game_status.hpp>
#include <rm_ros2_msgs/msg/game_robot_hp.hpp>
#include <rm_ros2_msgs/msg/power_heat_data.hpp>
// #include <rm_ros2_msgs/msg/ActuatorState.hpp>
#include <rm_ros2_msgs/msg/game_robot_status.hpp>
// #include <rm_msgs/ManualToReferee.h>
#include <rm_ros2_msgs/msg/power_management_sample_and_status_data.hpp>
#include "rm_ros2_manual/core/input_event.hpp"

namespace rm_ros2_manual
{
class ManualBase
{
public:
  explicit ManualBase(const rclcpp::Node::SharedPtr& node);
  virtual ~ManualBase() = default;
  enum
  {
    PASSIVE,
    IDLE,
    RC,
    PC
  };
  virtual void run();

protected:
  virtual void registerPubAndSub();
  virtual void checkReferee();
  virtual void checkKeyboard(const rm_ros2_msgs::msg::DbusData::SharedPtr& /*data*/) {};
  virtual void updateRc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data);
  virtual void updatePc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data);
  virtual void sendCommand(const rclcpp::Time& /*time*/) {};
  // virtual void updateActuatorStamp(const rm_msgs::ActuatorState::ConstPtr& data, std::vector<std::string> act_vector,
  // ros::Time& last_get_stamp);

  // Referee
  virtual void chassisOutputOn()
  {
    RCLCPP_INFO(node_->get_logger(), "Chassis output ON");
  }
  virtual void gimbalOutputOn()
  {
    RCLCPP_INFO(node_->get_logger(), "Gimbal output ON");
  }
  virtual void shooterOutputOn()
  {
    RCLCPP_INFO(node_->get_logger(), "Shooter output ON");
  }
  virtual void robotDie();
  virtual void robotRevive();

  // Remote Controller
  virtual void remoteControlTurnOff();
  virtual void remoteControlTurnOn();
  virtual void leftSwitchDownRise() {};
  virtual void leftSwitchMidRise() {};
  virtual void leftSwitchMidFall() {};
  virtual void leftSwitchUpRise() {};
  virtual void rightSwitchDownRise()
  {
    state_ = IDLE;
  }
  virtual void rightSwitchDownOn()
  {
    state_ = IDLE;
  }
  virtual void rightSwitchMidRise()
  {
    state_ = RC;
  }
  virtual void rightSwitchMidOn()
  {
    state_ = RC;
  }
  virtual void rightSwitchUpRise()
  {
    state_ = PC;
  }
  virtual void rightSwitchUpOn()
  {
    state_ = PC;
  }

  // CallBack
  virtual void dbusDataCallback(const rm_ros2_msgs::msg::DbusData::SharedPtr data);
  virtual void gameRobotStatusCallback(const rm_ros2_msgs::msg::GameRobotStatus::SharedPtr data);
  virtual void gameRobotHpCallback(const rm_ros2_msgs::msg::GameRobotHp::SharedPtr /*data*/)
  {
  }
  virtual void gameStatusCallback(const rm_ros2_msgs::msg::GameStatus::SharedPtr /*data*/)
  {
  }
  virtual void powerHeatDataCallback(const rm_ros2_msgs::msg::PowerHeatData::SharedPtr data);
  virtual void capacityDataCallback(const rm_ros2_msgs::msg::PowerManagementSampleAndStatusData::SharedPtr /*data*/)
  {
  }

  rclcpp::Node::SharedPtr node_;

  // Publishers
  // rclcpp::Publisher<rm_msgs::msg::ManualToReferee>::SharedPtr manual_to_referee_pub_;

  // Subscribers
  rclcpp::Subscription<rm_ros2_msgs::msg::DbusData>::SharedPtr dbus_sub_;
  rclcpp::Subscription<rm_ros2_msgs::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<rm_ros2_msgs::msg::GameRobotHp>::SharedPtr game_robot_hp_sub_;
  rclcpp::Subscription<rm_ros2_msgs::msg::GameRobotStatus>::SharedPtr game_robot_status_sub_;
  rclcpp::Subscription<rm_ros2_msgs::msg::PowerHeatData>::SharedPtr power_heat_data_sub_;
  rclcpp::Subscription<rm_ros2_msgs::msg::PowerManagementSampleAndStatusData>::SharedPtr capacity_sub_;

  // Data members
  // sensor_msgs::msg::JointState joint_state_;
  // rm_msgs::msg::ManualToReferee manual_to_referee_pub_data_;

  // Controller Manager
  std::shared_ptr<rm_ros2_common::ControllerManager> controller_manager_;

  // Timestamps and flags
  rclcpp::Time referee_last_get_stamp_;
  bool remote_is_open_{ false }, referee_is_online_ = false;
  int state_ = PASSIVE;
  int robot_id_, chassis_power_;
  int chassis_output_on_ = 0, gimbal_output_on_ = 0, shooter_output_on_ = 0;

  // Input Events
  InputEvent robot_hp_event_, right_switch_down_event_, right_switch_mid_event_, right_switch_up_event_,
      left_switch_down_event_, left_switch_mid_event_, left_switch_up_event_;
  InputEvent chassis_power_on_event_, gimbal_power_on_event_, shooter_power_on_event_;

  // Actuator timestamps
  rclcpp::Time chassis_actuator_last_get_stamp_, gimbal_actuator_last_get_stamp_, shooter_actuator_last_get_stamp_;

  // Motor names
  std::vector<std::string> chassis_mount_motor_, gimbal_mount_motor_, shooter_mount_motor_;
};
}  // namespace rm_ros2_manual

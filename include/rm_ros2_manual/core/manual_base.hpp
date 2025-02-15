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
  // virtual void checkReferee();
  virtual void checkKeyboard(const rm_ros2_msgs::msg::DbusData::SharedPtr& /*data*/) {};
  virtual void updateRc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data);
  virtual void updatePc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data);
  virtual void sendCommand(const rclcpp::Time& /*time*/) {};
  // virtual void updateActuatorStamp(const rm_msgs::ActuatorState::ConstPtr& data, std::vector<std::string> act_vector,
  // ros::Time& last_get_stamp);

  // virtual void jointStateCallback(const sensor_msgs::JointState::ConstPtr& data);
  virtual void dbusDataCallback(const rm_ros2_msgs::msg::DbusData::SharedPtr data);
  // virtual void trackCallback(const rm_msgs::TrackData::ConstPtr& data);
  // virtual void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data);
  // virtual void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data);
  // virtual void capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData::ConstPtr& data)
  // {
  // }
  // virtual void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data)
  // {
  // }
  // virtual void shootBeforehandCmdCallback(const rm_msgs::ShootBeforehandCmd ::ConstPtr& data)
  // {
  // }
  // virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
  // {
  // }
  // virtual void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data);
  // virtual void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
  // {
  // }
  // virtual void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
  // {
  // }
  // virtual void suggestFireCallback(const std_msgs::Bool::ConstPtr& data)
  // {
  // }
  // virtual void shootDataCallback(const rm_msgs::ShootData::ConstPtr& data)
  // {
  // }

  // Referee
  // virtual void chassisOutputOn()
  // {
  // ROS_INFO("Chassis output ON");
  // }
  // virtual void gimbalOutputOn()
  // {
  // ROS_INFO("Gimbal output ON");
  // }
  // virtual void shooterOutputOn()
  // {
  // ROS_INFO("Shooter output ON");
  // }
  // virtual void robotDie();
  // virtual void robotRevive();

  // Remote Controller
  virtual void remoteControlTurnOff() {};
  virtual void remoteControlTurnOn() {};
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

protected:
  rclcpp::Node::SharedPtr node_;

  // Publishers
  // rclcpp::Publisher<rm_msgs::msg::ManualToReferee>::SharedPtr manual_to_referee_pub_;

  // Subscribers
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rm_ros2_msgs::msg::DbusData>::SharedPtr dbus_sub_;
  // rclcpp::Subscription<rm_msgs::msg::TrackData>::SharedPtr track_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr referee_sub_;
  // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr capacity_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr game_status_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr game_robot_hp_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr actuator_state_sub_;
  // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr power_heat_data_sub_;
  // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_des_error_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr game_robot_status_sub_;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr suggest_fire_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shoot_beforehand_cmd_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shoot_data_sub_;

  // Data members
  // sensor_msgs::msg::JointState joint_state_;
  // rm_ros2_msgs::msg::TrackData track_data_;
  // rm_msgs::msg::ManualToReferee manual_to_referee_pub_data_;

  // Controller Manager
  // rm_common::ControllerManager controller_manager_;

  // TF2 Buffer and Listener
  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;

  // Timestamps and flags
  // rclcpp::Time referee_last_get_stamp_;
  bool remote_is_open_{ false }, referee_is_online_ = false;
  int state_ = PASSIVE;
  // int robot_id_, chassis_power_;
  // int chassis_output_on_ = 0, gimbal_output_on_ = 0, shooter_output_on_ = 0;

  // Input Events
  InputEvent robot_hp_event_, right_switch_down_event_, right_switch_mid_event_, right_switch_up_event_,
      left_switch_down_event_, left_switch_mid_event_, left_switch_up_event_;
  // InputEvent chassis_power_on_event_, gimbal_power_on_event_, shooter_power_on_event_;

  // Actuator timestamps
  // rclcpp::Time chassis_actuator_last_get_stamp_, gimbal_actuator_last_get_stamp_, shooter_actuator_last_get_stamp_;

  // Motor names
  // std::vector<std::string> chassis_mount_motor_, gimbal_mount_motor_, shooter_mount_motor_;
};
}  // namespace rm_ros2_manual

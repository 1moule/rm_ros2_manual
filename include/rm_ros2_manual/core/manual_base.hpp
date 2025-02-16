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
  virtual void checkKeyboard(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data);
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
  // PC
  virtual void ctrlAPress() {};
  virtual void ctrlBPress() {};
  virtual void ctrlCPress() {};
  virtual void ctrlDPress() {};
  virtual void ctrlEPress() {};
  virtual void ctrlFPress() {};
  virtual void ctrlGPress() {};
  virtual void ctrlQPress() {};
  virtual void ctrlRPress() {};
  virtual void ctrlSPress() {};
  virtual void ctrlVPress() {};
  virtual void ctrlVRelease() {};
  virtual void ctrlWPress() {};
  virtual void ctrlXPress() {};
  virtual void ctrlZPress() {};

  virtual void aPress() {};
  virtual void aPressing() {};
  virtual void aRelease() {};
  virtual void bPressing() {};
  virtual void bRelease() {};
  virtual void cPressing() {};
  virtual void cRelease() {};
  virtual void dPress() {};
  virtual void dPressing() {};
  virtual void dRelease() {};
  virtual void ePressing() {};
  virtual void eRelease() {};
  virtual void fPress() {};
  virtual void fRelease() {};
  virtual void gPress() {};
  virtual void gRelease() {};
  virtual void qPressing() {};
  virtual void qRelease() {};
  virtual void rPress() {};
  virtual void sPress() {};
  virtual void sPressing() {};
  virtual void sRelease() {};
  virtual void vPressing() {};
  virtual void vRelease() {};
  virtual void wPress() {};
  virtual void wPressing() {};
  virtual void wRelease() {};
  virtual void xPress() {};
  virtual void zPressing() {};
  virtual void zRelease() {};

  virtual void shiftPressing() {};
  virtual void shiftRelease() {};
  virtual void shiftBPress() {};
  virtual void shiftBRelease() {};
  virtual void shiftCPress() {};
  virtual void shiftEPress() {};
  virtual void shiftFPress() {};
  virtual void shiftGPress() {};
  virtual void shiftQPress() {};
  virtual void shiftRPress() {};
  virtual void shiftRRelease() {};
  virtual void shiftVPress() {};
  virtual void shiftVRelease() {};
  virtual void shiftXPress() {};
  virtual void shiftZPress() {};
  virtual void shiftZRelease() {};

  virtual void mouseLeftRelease() {};
  virtual void mouseRightRelease() {};
  virtual void mouseMidRise(double m_z) {};

  // CallBack
  virtual void dbusDataCallback(const rm_ros2_msgs::msg::DbusData::SharedPtr data);
  virtual void gameRobotStatusCallback(const rm_ros2_msgs::msg::GameRobotStatus::SharedPtr data);
  virtual void gameRobotHpCallback(const rm_ros2_msgs::msg::GameRobotHp::SharedPtr /*data*/) {};
  virtual void gameStatusCallback(const rm_ros2_msgs::msg::GameStatus::SharedPtr /*data*/) {};
  virtual void powerHeatDataCallback(const rm_ros2_msgs::msg::PowerHeatData::SharedPtr data);
  virtual void capacityDataCallback(const rm_ros2_msgs::msg::PowerManagementSampleAndStatusData::SharedPtr /*data*/) {};

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
  // rm_msgs::msg::ManualToReferee manual_to_referee_pub_data_;

  // Controller Manager
  std::shared_ptr<rm_ros2_common::ControllerManager> controller_manager_;

  // Timestamps and flags
  rclcpp::Time referee_last_get_stamp_;
  bool remote_is_open_{ false }, referee_is_online_ = false;
  double chassis_power_{};
  int state_ = PASSIVE;
  int robot_id_ = 0;
  int chassis_output_on_ = 0, gimbal_output_on_ = 0, shooter_output_on_ = 0;

  // Input Events
  InputEvent right_switch_down_event_, right_switch_mid_event_, right_switch_up_event_, left_switch_down_event_,
      left_switch_mid_event_, left_switch_up_event_;
  InputEvent a_event_, b_event_, c_event_, d_event_, e_event_, f_event_, g_event_, q_event_, r_event_, s_event_,
      v_event_, w_event_, x_event_, z_event_;
  InputEvent ctrl_a_event_, ctrl_b_event_, ctrl_c_event_, ctrl_d_event_, ctrl_e_event_, ctrl_f_event_, ctrl_g_event_,
      ctrl_q_event_, ctrl_r_event_, ctrl_s_event_, ctrl_v_event_, ctrl_w_event_, ctrl_x_event_, ctrl_z_event_;
  InputEvent shift_event_, shift_a_event_, shift_b_event_, shift_c_event_, shift_d_event_, shift_e_event_,
      shift_f_event_, shift_g_event_, shift_q_event_, shift_r_event_, shift_s_event_, shift_v_event_, shift_x_event_,
      shift_z_event_;
  InputEvent mouse_left_event_, mouse_right_event_;
  InputEvent chassis_power_on_event_, gimbal_power_on_event_, shooter_power_on_event_, robot_hp_event_;

  // Actuator timestamps
  rclcpp::Time chassis_actuator_last_get_stamp_, gimbal_actuator_last_get_stamp_, shooter_actuator_last_get_stamp_;

  // Motor names
  std::vector<std::string> chassis_mount_motor_, gimbal_mount_motor_, shooter_mount_motor_;
};
}  // namespace rm_ros2_manual

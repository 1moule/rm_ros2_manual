//
// Created by guanlin on 25-2-15.
//

#pragma once

#include <rm_ros2_manual/core/manual_base.hpp>
#include <rm_ros2_common/decision/command_sender.hpp>

namespace rm_ros2_manual
{
class ChassisGimbalManual : public ManualBase
{
public:
  explicit ChassisGimbalManual(const rclcpp::Node::SharedPtr& node);

protected:
  void sendCommand(const rclcpp::Time& time) override;
  void updateRc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data) override;
  void updatePc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data) override;
  // void checkReferee() override;
  void checkKeyboard(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data) override;
  void remoteControlTurnOff() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchDownRise() override;
  void dbusDataCallback(const rm_ros2_msgs::msg::DbusData::SharedPtr data) override;
  // void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  // void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  // void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data) override;
  // void capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData ::ConstPtr& data) override;
  // void trackCallback(const rm_msgs::TrackData::ConstPtr& data) override;
  void setChassisMode(int mode);
  virtual void wPress()
  {
    x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  }
  virtual void aPress()
  {
    y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  }
  virtual void sPress()
  {
    x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  }
  virtual void dPress()
  {
    y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  }
  virtual void wRelease();
  virtual void sRelease();
  virtual void aRelease();
  virtual void dRelease();
  virtual void wPressing();
  virtual void sPressing();
  virtual void aPressing();
  virtual void dPressing();
  virtual void mouseMidRise(double m_z);

  std::shared_ptr<rm_ros2_common::Vel2DCommandSender> vel_cmd_sender_;
  std::shared_ptr<rm_ros2_common::GimbalCommandSender> gimbal_cmd_sender_;
  std::shared_ptr<rm_ros2_common::ChassisCommandSender> chassis_cmd_sender_;

  double x_scale_{}, y_scale_{};
  bool is_gyro_{};
  double gimbal_scale_{ 1. };
  double gyro_move_reduction_{ 1. };
  double gyro_rotate_reduction_{ 1. };

  InputEvent w_event_, s_event_, a_event_, d_event_;
};
}  // namespace rm_ros2_manual

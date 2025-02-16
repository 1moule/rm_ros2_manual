//
// Created by guanlin on 25-2-15.
//

#include "rm_ros2_manual/core/chassis_gimbal_manual.hpp"

namespace rm_ros2_manual
{
ChassisGimbalManual::ChassisGimbalManual(const rclcpp::Node::SharedPtr& node) : ManualBase(node)
{
  chassis_cmd_sender_ = std::make_shared<rm_ros2_common::ChassisCommandSender>(node_, "chassis");

  vel_cmd_sender_ = std::make_shared<rm_ros2_common::Vel2DCommandSender>(node_, "vel");
  gyro_move_reduction_ = getParam(node_, "vel.gyro_move_reduction", 1.0);
  gyro_rotate_reduction_ = getParam(node_, "vel.gyro_rotate_reduction", 1.0);

  gimbal_cmd_sender_ = std::make_shared<rm_ros2_common::GimbalCommandSender>(node_, "gimbal");
  gimbal_scale_ = getParam(node_, "gimbal.gimbal_scale", 1.0);

  // chassis_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::chassisOutputOn, this));
  // gimbal_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::gimbalOutputOn, this));
}

void ChassisGimbalManual::sendCommand(const rclcpp::Time& time)
{
  chassis_cmd_sender_->sendChassisCommand(time, is_gyro_);
  vel_cmd_sender_->sendCommand(time);
  gimbal_cmd_sender_->sendCommand(time);
}

void ChassisGimbalManual::updateRc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  gimbal_cmd_sender_->setRate(-dbus_data->ch_l_x, -dbus_data->ch_l_y);
  if (gimbal_cmd_sender_->getMsg()->mode == rm_ros2_msgs::msg::GimbalCmd::RATE)
    chassis_cmd_sender_->setFollowVelDes(gimbal_cmd_sender_->getMsg()->rate_yaw);
  else
    chassis_cmd_sender_->setFollowVelDes(0.);
}
void ChassisGimbalManual::updatePc(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  gimbal_cmd_sender_->setRate(-dbus_data->m_x * gimbal_scale_, dbus_data->m_y * gimbal_scale_);
  if (gimbal_cmd_sender_->getMsg()->mode == rm_ros2_msgs::msg::GimbalCmd::RATE)
    chassis_cmd_sender_->setFollowVelDes(gimbal_cmd_sender_->getMsg()->rate_yaw);
  else
    chassis_cmd_sender_->setFollowVelDes(0.);
}

// void ChassisGimbalManual::checkReferee()
// {
// ManualBase::checkReferee();
// }

void ChassisGimbalManual::checkKeyboard(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data)
{
  w_event_.update((!dbus_data->key_ctrl) && dbus_data->key_w);
  s_event_.update((!dbus_data->key_ctrl) && dbus_data->key_s);
  a_event_.update((!dbus_data->key_ctrl) && dbus_data->key_a);
  d_event_.update((!dbus_data->key_ctrl) && dbus_data->key_d);
  if (dbus_data->m_z != 0)
    mouseMidRise(dbus_data->m_z);
}

// void ChassisGimbalManual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
// {
// ManualBase::gameStatusCallback(data);
// chassis_cmd_sender_->updateGameStatus(*data);
// }

// void ChassisGimbalManual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
// {
// ManualBase::gameRobotStatusCallback(data);
// chassis_cmd_sender_->updateGameRobotStatus(*data);
// }

// void ChassisGimbalManual::powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
// {
// ManualBase::powerHeatDataCallback(data);
// chassis_cmd_sender_->updatePowerHeatData(*data);
// }

void ChassisGimbalManual::dbusDataCallback(const rm_ros2_msgs::msg::DbusData::SharedPtr data)
{
  ManualBase::dbusDataCallback(data);
  // chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
}

// void ChassisGimbalManual::capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData ::ConstPtr& data)
// {
// ManualBase::capacityDataCallback(data);
// chassis_cmd_sender_->updateCapacityData(*data);
// }

// void ChassisGimbalManual::trackCallback(const rm_msgs::TrackData::ConstPtr& data)
// {
// ManualBase::trackCallback(data);
// }

void ChassisGimbalManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  chassis_cmd_sender_->setZero();
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchDownRise()
{
  ManualBase::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_ros2_msgs::msg::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_ros2_msgs::msg::GimbalCmd::RATE);
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_ros2_msgs::msg::ChassisCmd::FOLLOW);
  gimbal_cmd_sender_->setMode(rm_ros2_msgs::msg::GimbalCmd::RATE);
}

void ChassisGimbalManual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_ros2_msgs::msg::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_ros2_msgs::msg::GimbalCmd::RATE);
}

void ChassisGimbalManual::leftSwitchDownRise()
{
  ManualBase::leftSwitchDownRise();
  gimbal_cmd_sender_->setMode(rm_ros2_msgs::msg::GimbalCmd::RATE);
}

void ChassisGimbalManual::wRelease()
{
  vel_cmd_sender_->setLinearXVel(x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0);
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0);
}

void ChassisGimbalManual::sRelease()
{
  vel_cmd_sender_->setLinearXVel(x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0);
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0);
}

void ChassisGimbalManual::aRelease()
{
  vel_cmd_sender_->setLinearYVel(y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0);
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0);
}

void ChassisGimbalManual::dRelease()
{
  vel_cmd_sender_->setLinearYVel(y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0);
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0);
}

void ChassisGimbalManual::wPressing()
{
  vel_cmd_sender_->setLinearXVel(is_gyro_ ? gyro_move_reduction_ : x_scale_);
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalManual::sPressing()
{
  vel_cmd_sender_->setLinearXVel(is_gyro_ ? gyro_move_reduction_ : x_scale_);
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalManual::aPressing()
{
  vel_cmd_sender_->setLinearYVel(is_gyro_ ? gyro_move_reduction_ : y_scale_);
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalManual::dPressing()
{
  vel_cmd_sender_->setLinearYVel(is_gyro_ ? gyro_move_reduction_ : y_scale_);
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalManual::mouseMidRise(double m_z)
{
  if (gimbal_scale_ >= 1. && gimbal_scale_ <= 30.)
  {
    if (gimbal_scale_ + 1. <= 30. && m_z > 0.)
      gimbal_scale_ += 1.;
    else if (gimbal_scale_ - 1. >= 1. && m_z < 0.)
      gimbal_scale_ -= 1.;
  }
}

void ChassisGimbalManual::setChassisMode(int mode)
{
  switch (mode)
  {
    case rm_ros2_msgs::msg::ChassisCmd::RAW:
      chassis_cmd_sender_->setMode(rm_ros2_msgs::msg::ChassisCmd::RAW);
      is_gyro_ = true;
      if (x_scale_ != 0.0 || y_scale_ != 0.0)
        vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_);
      else
        vel_cmd_sender_->setAngularZVel(1.0);
      break;
    case rm_ros2_msgs::msg::ChassisCmd::FOLLOW:
      chassis_cmd_sender_->setMode(rm_ros2_msgs::msg::ChassisCmd::FOLLOW);
      is_gyro_ = false;
      vel_cmd_sender_->setAngularZVel(0.0);
      break;
    default:
      break;
  }
}
}  // namespace rm_ros2_manual

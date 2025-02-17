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
  controller_manager_ = std::make_shared<rm_ros2_common::ControllerManager>();
  // RC
  right_switch_down_event_.setRising([this] { rightSwitchDownRise(); });
  right_switch_mid_event_.setRising([this] { rightSwitchMidRise(); });
  right_switch_up_event_.setRising([this] { rightSwitchUpRise(); });
  right_switch_down_event_.setActiveHigh([this] { rightSwitchDownOn(); });
  right_switch_mid_event_.setActiveHigh([this] { rightSwitchMidOn(); });
  right_switch_up_event_.setActiveHigh([this] { rightSwitchUpOn(); });
  left_switch_down_event_.setRising([this] { leftSwitchDownRise(); });
  left_switch_up_event_.setRising([this] { leftSwitchUpRise(); });
  left_switch_mid_event_.setEdge([this] { leftSwitchMidRise(); }, [this] { leftSwitchMidFall(); });
  // PC
  ctrl_a_event_.setRising([this] { ctrlAPress(); });
  ctrl_b_event_.setRising([this] { ctrlBPress(); });
  ctrl_c_event_.setRising([this] { ctrlCPress(); });
  ctrl_d_event_.setRising([this] { ctrlDPress(); });
  ctrl_e_event_.setRising([this] { ctrlEPress(); });
  ctrl_f_event_.setRising([this] { ctrlFPress(); });
  ctrl_g_event_.setRising([this] { ctrlGPress(); });
  ctrl_q_event_.setRising([this] { ctrlQPress(); });
  ctrl_r_event_.setRising([this] { ctrlRPress(); });
  ctrl_s_event_.setRising([this] { ctrlSPress(); });
  ctrl_v_event_.setRising([this] { ctrlVPress(); });
  ctrl_v_event_.setFalling([this] { ctrlVRelease(); });
  ctrl_w_event_.setRising([this] { ctrlWPress(); });
  ctrl_x_event_.setRising([this] { ctrlXPress(); });
  ctrl_z_event_.setRising([this] { ctrlZPress(); });
  a_event_.setEdge([this] { aPress(); }, [this] { aRelease(); });
  a_event_.setActiveHigh([this] { aPressing(); });
  b_event_.setActiveHigh([this] { bPressing(); });
  b_event_.setFalling([this] { bRelease(); });
  c_event_.setActiveHigh([this] { cPressing(); });
  c_event_.setFalling([this] { cRelease(); });
  d_event_.setEdge([this] { dPress(); }, [this] { dRelease(); });
  d_event_.setActiveHigh([this] { dPressing(); });
  e_event_.setActiveHigh([this] { ePressing(); });
  e_event_.setFalling([this] { eRelease(); });
  f_event_.setRising([this] { fPress(); });
  f_event_.setFalling([this] { fRelease(); });
  g_event_.setRising([this] { gPress(); });
  g_event_.setFalling([this] { gRelease(); });
  q_event_.setActiveHigh([this] { qPressing(); });
  q_event_.setFalling([this] { qRelease(); });
  r_event_.setRising([this] { rPress(); });
  s_event_.setEdge([this] { sPress(); }, [this] { sRelease(); });
  s_event_.setActiveHigh([this] { sPressing(); });
  v_event_.setActiveHigh([this] { vPressing(); });
  v_event_.setFalling([this] { vRelease(); });
  w_event_.setEdge([this] { wPress(); }, [this] { wRelease(); });
  w_event_.setActiveHigh([this] { wPressing(); });
  x_event_.setRising([this] { xPress(); });
  z_event_.setActiveHigh([this] { zPressing(); });
  z_event_.setFalling([this] { zRelease(); });
  shift_event_.setActiveHigh([this] { shiftPressing(); });
  shift_event_.setFalling([this] { shiftRelease(); });
  shift_b_event_.setRising([this] { shiftBPress(); });
  shift_b_event_.setFalling([this] { shiftBRelease(); });
  shift_c_event_.setRising([this] { shiftCPress(); });
  shift_e_event_.setRising([this] { shiftEPress(); });
  shift_f_event_.setRising([this] { shiftFPress(); });
  shift_g_event_.setRising([this] { shiftGPress(); });
  shift_q_event_.setRising([this] { shiftQPress(); });
  shift_r_event_.setRising([this] { shiftRPress(); });
  shift_r_event_.setFalling([this] { shiftRRelease(); });
  shift_v_event_.setRising([this] { shiftVPress(); });
  shift_v_event_.setFalling([this] { shiftVRelease(); });
  shift_x_event_.setRising([this] { shiftXPress(); });
  shift_z_event_.setRising([this] { shiftZPress(); });
  shift_z_event_.setFalling([this] { shiftZRelease(); });

  mouse_left_event_.setFalling([this] { mouseLeftRelease(); });
  mouse_right_event_.setFalling([this] { mouseRightRelease(); });
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

void ManualBase::checkKeyboard(const rm_ros2_msgs::msg::DbusData::SharedPtr& dbus_data)
{
  ctrl_a_event_.update(dbus_data->key_ctrl & dbus_data->key_a);
  ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b);
  ctrl_c_event_.update(dbus_data->key_ctrl & dbus_data->key_c);
  ctrl_d_event_.update(dbus_data->key_ctrl & dbus_data->key_d);
  ctrl_e_event_.update(dbus_data->key_ctrl & dbus_data->key_e);
  ctrl_f_event_.update(dbus_data->key_ctrl & dbus_data->key_f);
  ctrl_g_event_.update(dbus_data->key_ctrl & dbus_data->key_g);
  ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);
  ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
  ctrl_s_event_.update(dbus_data->key_ctrl & dbus_data->key_s);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
  ctrl_w_event_.update(dbus_data->key_ctrl & dbus_data->key_w);
  ctrl_x_event_.update(dbus_data->key_ctrl & dbus_data->key_x);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);

  a_event_.update(dbus_data->key_a & !dbus_data->key_ctrl & !dbus_data->key_shift);
  b_event_.update(dbus_data->key_b & !dbus_data->key_ctrl & !dbus_data->key_shift);
  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
  d_event_.update(dbus_data->key_d & !dbus_data->key_ctrl & !dbus_data->key_shift);
  e_event_.update(dbus_data->key_e & !dbus_data->key_ctrl & !dbus_data->key_shift);
  f_event_.update(dbus_data->key_f & !dbus_data->key_ctrl & !dbus_data->key_shift);
  g_event_.update(dbus_data->key_g & !dbus_data->key_ctrl & !dbus_data->key_shift);
  q_event_.update(dbus_data->key_q & !dbus_data->key_ctrl & !dbus_data->key_shift);
  r_event_.update(dbus_data->key_r & !dbus_data->key_ctrl & !dbus_data->key_shift);
  s_event_.update(dbus_data->key_s & !dbus_data->key_ctrl & !dbus_data->key_shift);
  v_event_.update(dbus_data->key_v & !dbus_data->key_ctrl & !dbus_data->key_shift);
  w_event_.update(dbus_data->key_w & !dbus_data->key_ctrl & !dbus_data->key_shift);
  x_event_.update(dbus_data->key_x & !dbus_data->key_ctrl & !dbus_data->key_shift);
  z_event_.update(dbus_data->key_z & !dbus_data->key_ctrl & !dbus_data->key_shift);

  shift_event_.update(dbus_data->key_shift & !dbus_data->key_ctrl);
  shift_b_event_.update(dbus_data->key_shift & dbus_data->key_b);
  shift_c_event_.update(dbus_data->key_shift & dbus_data->key_c);
  shift_e_event_.update(dbus_data->key_shift & dbus_data->key_e);
  shift_f_event_.update(dbus_data->key_shift & dbus_data->key_f);
  shift_g_event_.update(dbus_data->key_shift & dbus_data->key_g);
  shift_q_event_.update(dbus_data->key_shift & dbus_data->key_q);
  shift_r_event_.update(dbus_data->key_shift & dbus_data->key_r);
  shift_v_event_.update(dbus_data->key_shift & dbus_data->key_v);
  shift_x_event_.update(dbus_data->key_shift & dbus_data->key_x);
  shift_z_event_.update(dbus_data->key_shift & dbus_data->key_z);

  mouse_left_event_.update(dbus_data->p_l);
  mouse_right_event_.update(dbus_data->p_r);
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
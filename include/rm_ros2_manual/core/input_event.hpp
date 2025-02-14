//
// Created by guanlin on 25-2-14.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <utility>

namespace rm_ros2_manual
{
class InputEvent
{
public:
  InputEvent() : last_state_(false)
  {
  }

  bool getState() const
  {
    return last_state_;
  }

  void setRising(std::function<void()> handler)
  {
    rising_handler_ = std::move(handler);
  }

  void setFalling(std::function<void()> handler)
  {
    falling_handler_ = std::move(handler);
  }

  void setActiveHigh(std::function<void(rclcpp::Duration)> handler)
  {
    active_high_handler_ = std::move(handler);
  }

  void setActiveLow(std::function<void(rclcpp::Duration)> handler)
  {
    active_low_handler_ = std::move(handler);
  }

  void setEdge(std::function<void()> rising_handler, std::function<void()> falling_handler)
  {
    rising_handler_ = std::move(rising_handler);
    falling_handler_ = std::move(falling_handler);
  }

  void setActive(std::function<void(rclcpp::Duration)> high_handler, std::function<void(rclcpp::Duration)> low_handler)
  {
    active_high_handler_ = std::move(high_handler);
    active_low_handler_ = std::move(low_handler);
  }

  void setDelayTriggered(std::function<void()> delay_handler, double duration, bool is_rising_trigger)
  {
    const rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("input_event_node");
    delay_time_ = duration;
    const auto callback_group = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    triggered_timer_ =
        nh->create_wall_timer(std::chrono::duration<double>(delay_time_), std::move(delay_handler), callback_group);
    if (is_rising_trigger)
      trigger_rising_handler_ = [this] { startTimer(); };
    else
      trigger_falling_handler_ = [this] { startTimer(); };
  }

  void update(bool state)
  {
    const auto now = rclcpp::Clock().now();
    if (state != last_state_)
    {
      if (state && rising_handler_)
        rising_handler_();
      else if (!state && falling_handler_)
        falling_handler_();
      if (state && trigger_rising_handler_)
        trigger_rising_handler_();
      else if (!state && trigger_falling_handler_)
        trigger_falling_handler_();
      last_state_ = state;
      last_change_ = now;
    }
    if (state && active_high_handler_)
      active_high_handler_(now - last_change_);
    if (!state && active_low_handler_)
      active_low_handler_(now - last_change_);
  }

private:
  void startTimer() const
  {
    triggered_timer_->reset();
  }

  bool last_state_;
  double delay_time_{};
  rclcpp::Time last_change_;
  rclcpp::TimerBase::SharedPtr triggered_timer_;
  std::function<void(rclcpp::Duration)> active_high_handler_, active_low_handler_;
  std::function<void()> rising_handler_, falling_handler_, trigger_rising_handler_, trigger_falling_handler_;
};

}  // namespace rm_ros2_manual

//
// Created by guanlin on 25-2-12.
//

#include "rm_ros2_manual/standard_manual.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rm_manual");
  auto node_referee = node->create_sub_node("rm_referee");

  std::string robot;
  node->declare_parameter<std::string>("robot_type", "error");
  node->get_parameter("robot_type", robot);

  std::shared_ptr<rm_ros2_manual::ManualBase> manual_control;

  // if (robot == "standard")
  //   manual_control = std::make_shared<rm_ros2_manual::ChassisGimbalShooterCoverManual>(nh, nh_referee);
  // else if ((robot == "hero") || (robot == "drone"))
  //   manual_control = std::make_shared<rm_ros2_manual::ChassisGimbalShooterManual>(nh, nh_referee);
  // else if (robot == "engineer")
  //   manual_control = std::make_shared<rm_ros2_manual::EngineerManual>(nh, nh_referee);
  // else if (robot == "engineer2")
  //   manual_control = std::make_shared<rm_ros2_manual::Engineer2Manual>(nh, nh_referee);
  // else if (robot == "dart")
  //   manual_control = std::make_shared<rm_ros2_manual::DartManual>(nh, nh_referee);
  // else if (robot == "balance")
  //   manual_control = std::make_shared<rm_ros2_manual::BalanceManual>(nh, nh_referee);
  // else
  // {
  //   RCLCPP_ERROR(nh->get_logger(), "No robot type specified");
  //   return 0;
  // }
  manual_control = std::make_shared<rm_ros2_manual::StandardManual>(node);

  rclcpp::Rate loop_rate(100);
  while (rclcpp::ok())
  {
    spin_some(node);
    manual_control->run();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
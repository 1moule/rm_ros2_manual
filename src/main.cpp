//
// Created by guanlin on 25-2-12.
//

#include "rm_ros2_manual/lifecycle_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm_ros2_manual::LifecycleNode>("rm_ros2_manual");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
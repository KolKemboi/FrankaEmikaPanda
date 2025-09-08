#include <iostream>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <utility>
//
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "Panda_Control",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success) {
    move_group_interface.execute(plan);
  } else {
    std::cerr << "Failed" << std::endl;
  }
  auto const logger = rclcpp::get_logger("Panda Arm");

  rclcpp::shutdown();
}

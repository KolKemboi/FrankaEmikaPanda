#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <shape_msgs/msg/detail/solid_primitive__struct.hpp>
#include <thread>
#include <utility>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "Panda_Control",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  auto const logger = rclcpp::get_logger("Panda_Logger");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.y = 0.8;
    msg.orientation.w = 0.6;
    msg.position.x = 0.1;
    msg.position.y = 0.05;
    msg.position.z = 0.3;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  auto const coll_obj_1 = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject coll_obj_1;
    coll_obj_1.header.frame_id = frame_id;
    coll_obj_1.id = "box_1";
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    coll_obj_1.primitives.push_back(primitive);
    coll_obj_1.primitive_poses.push_back(box_pose);
    coll_obj_1.operation = coll_obj_1.ADD;

    return coll_obj_1;
  }();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(coll_obj_1);

  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Failed planning");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}

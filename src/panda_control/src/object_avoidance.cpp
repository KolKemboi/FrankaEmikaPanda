#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "Panda_Control",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

	auto const logger = rclcpp::get_logger("Panda_Logger");

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	auto spinner = std::thread([&executor]() {executor.spin();});

	using moveit::planning_interface::MoveGroupInterface;
	auto move_group_interface = MoveGroupInterface(node, "panda_arm");

	auto const target_pose = []{
	geometry_msgs::msg::Pose msg;
		msg.orientation.y = 0.8;
		msg.orientation.w = 0.6;
		msg.position.x = 0.1;
		msg.position.y = 0.05;
		msg.position.z = 0.3;
		return msg;
	}();
	move_group_interface.setPoseTarget(target_pose);

	auto const coll_obj_1 = [frame_id = move_group_interface.getPlanningFrame()]{

	}

  rclcpp::shutdown();
}

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <map>
#include <string>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("panda_mtc_two_movements");

    // Load robot model
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    auto kinematic_model = robot_model_loader.getModel();
    if (!kinematic_model) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load robot model");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Loaded robot model: %s", kinematic_model->getName().c_str());

    // Create the Task
    moveit::task_constructor::Task task("Two Movements Demo");
    task.setRobotModel(kinematic_model);

    // Add current state stage
    auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");
    task.add(std::move(current_state));

    // Create planner
    auto planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node);

    // Get the arm group
    auto arm_group = kinematic_model->getJointModelGroup("panda_arm");
    if (!arm_group) {
        RCLCPP_ERROR(node->get_logger(), "panda_arm group not found");
        return 1;
    }

    // Movement 1: Move to first position (conservative joint values)
    auto move_to_pos1 = std::make_unique<moveit::task_constructor::stages::MoveTo>("move_to_position_1", planner);
    move_to_pos1->setGroup("panda_arm");
    
    std::map<std::string, double> position_1 = {
        {"panda_joint1", 0.0},
        {"panda_joint2", -0.5},    // Shoulder
        {"panda_joint3", 0.0},
        {"panda_joint4", -1.0},    // Elbow
        {"panda_joint5", 0.0},
        {"panda_joint6", 1.0},     // Wrist
        {"panda_joint7", 0.5}      // Hand
    };
    move_to_pos1->setGoal(position_1);
    task.add(std::move(move_to_pos1));

    // Movement 2: Move to second position (slightly different)
    auto move_to_pos2 = std::make_unique<moveit::task_constructor::stages::MoveTo>("move_to_position_2", planner);
    move_to_pos2->setGroup("panda_arm");
    
    std::map<std::string, double> position_2 = {
        {"panda_joint1", 0.3},     // Slight rotation
        {"panda_joint2", -0.3},    // Shoulder up
        {"panda_joint3", 0.2},     // Slight bend
        {"panda_joint4", -1.2},    // Elbow down
        {"panda_joint5", 0.1},     // Slight rotation
        {"panda_joint6", 1.2},     // Wrist up
        {"panda_joint7", 0.7}      // Hand rotated
    };
    move_to_pos2->setGoal(position_2);
    task.add(std::move(move_to_pos2));

    try {
        // Initialize the task
        RCLCPP_INFO(node->get_logger(), "Initializing task...");
        task.init();
        
        RCLCPP_INFO(node->get_logger(), "Planning two movements...");
        
        // Plan the full task
        if (task.plan()) {
            RCLCPP_INFO(node->get_logger(), "Planning successful! Found %zu solutions", task.solutions().size());
            
            if (!task.solutions().empty()) {
                RCLCPP_INFO(node->get_logger(), "Executing first solution...");
                auto result = task.execute(*task.solutions().front());
                
                if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    RCLCPP_INFO(node->get_logger(), "Execution successful! Completed two movements.");
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Execution failed with error code: %d", result.val);
                }
            } else {
                RCLCPP_ERROR(node->get_logger(), "No solutions found");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Planning failed!");
            RCLCPP_INFO(node->get_logger(), "Task state:");
            task.printState();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "Task completed");
    rclcpp::shutdown();
    return 0;
}

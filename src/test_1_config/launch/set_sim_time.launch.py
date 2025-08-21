from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])

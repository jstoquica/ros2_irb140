from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    irb140_path = FindPackageShare("irb140")

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([irb140_path, "urdf/irb140.urdf.xacro"])
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                PathJoinSubstitution([irb140_path, "config/controllers.yaml"])
            ],
            output="screen"
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen"
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_trajectory_controller"],
                    output="screen"
                )
            ]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([irb140_path, "rviz/irb140.rviz"])],
            output="screen"
        )
    ])


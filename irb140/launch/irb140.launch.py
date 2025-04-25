# View and view the URDF in RViz

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, DeclareLaunchArgument



def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    robot_description_path = get_package_share_directory("irb140")

    robot_description_config = xacro.process_file(
        os.path.join(
            robot_description_path,
            "urdf",
            "irb140.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_sliders = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    controller = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=["src/irb140/config/controllers.yaml"],
        output="both",
    )

    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],   
    )

    trajectory = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller"],
        output="both",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(robot_description_path, "rviz", "irb140.rviz"),
        ],
        output="screen",
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=broadcaster,
            on_exit=[rviz],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller,
            on_exit=[broadcaster],
        )
    )

    return LaunchDescription([robot_state_publisher_node, joint_state_sliders, rviz, controller, broadcaster, trajectory])
#    return LaunchDescription([
#        robot_state_publisher_node, 
#        joint_state_sliders, 
#        controller, 
#        delay_rviz_after_joint_state_broadcaster_spawner, 
#        delay_joint_state_broadcaster_after_robot_controller_spawner])




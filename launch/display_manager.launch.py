#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import (LaunchConfiguration,
            EnvironmentVariable)
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    package_name = "vikings_bot_display_manager"

    ### INPUT ###
    vikings_bot_name_arg = DeclareLaunchArgument("vikings_bot_name",
                default_value=EnvironmentVariable("ROBOT_NAME"),
                description="Namespace of robot - [vikings_bot_1 or vikings_bot_2]"
    )
    net_interface_arg = DeclareLaunchArgument('net_interface',
                default_value="wlo1",
                description="Provide network interface to show data usage. Use ip link show to find out.")

    namespace = LaunchConfiguration("vikings_bot_name")
    net_interface = LaunchConfiguration("net_interface")

    display_manager_node = Node(
        package=package_name,
        executable="display_manager",
        namespace=namespace,
        name="display_manager",
        output="screen",
        parameters=[{
            "net_interface":net_interface
        }]
    )

    return LaunchDescription(
        [
            vikings_bot_name_arg,
            net_interface_arg,
            display_manager_node
        ]
    )

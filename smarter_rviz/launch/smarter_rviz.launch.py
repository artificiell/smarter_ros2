#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # Rviz node
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        arguments = [
            '-d', [PathJoinSubstitution([
                FindPackageShare('smarter_rviz'),
                'config',
                'turtlebot3.config.rviz'
            ])]
        ]
    )

    return LaunchDescription([
        rviz_node
    ])

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    xbox_joystick_node = Node(
        package='cool_robot_joystick',
        executable='xbox_joystick',
        name='joystick',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        xbox_joystick_node
    ])

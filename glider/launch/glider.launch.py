"""
    Jason Hughes
    January 2025

    Launch the factor graph node
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory("glider")

    return LaunchDescription([Node(package="glider",
                                   executable="glider_ros",
                                   name="glider_node",
                                   output="screen",
                                   emulate_tty=True,
                                   #arguments=['--ros-args', '--log-level', 'DEBUG'],
                                   remappings=[("/gps", "/ublox/fix"),
                                               ("/imu", "/VN100T/imu"),
                                               ("/odom", "/Odometry"),],
                                   parameters=[{"use_sim_time": False}])])

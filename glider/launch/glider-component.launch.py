"""
    Jason Hughes
    January 2025

    Launch the factor graph node
"""

import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory("glider")

    container = ComposableNodeContainer(
    name='glider',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='glider',
            plugin='glider::FactorManagerNode',
            name='glider_component',
            remappings=[("/gps", "/ublox/fix"), ("/imu", "/VN100T/imu"), ("/odom", "/Odometry")],
        ),],
    output='screen',
    )

    return LaunchDescription([container])

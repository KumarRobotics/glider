"""
    Jason Hughes
    January 2025

    Launch the factor graph node
"""

import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory("glider")

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_odom_arg = DeclareLaunchArgument(
        'use_odom',
        default_value='true',
        description='Use odometry'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_odom = LaunchConfiguration('use_odom')
    
    # Find package share directory
    glider_share = FindPackageShare('glider')
    
    # Path to parameter files
    ros_params_file = PathJoinSubstitution([
        glider_share,
        'config',
        'ros_params.yaml'
    ])
    
    graph_params_file = PathJoinSubstitution([
        glider_share,
        'config',
        'graph_params.yaml'
    ])

    container = ComposableNodeContainer(
    name='glider_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='glider',
            plugin='glider::FactorManager',
            name='glider_component',
            parameters=[ros_params_file,
                        {'path': graph_params_file,
                         'use_sim_time': use_sim_time,
                         'use_odom': use_odom}],
            remappings=[("/gps", "/ublox/fix"), ("/imu", "/VN100T/imu"), ("/odom", "/Odometry")],
        ),],
    output='screen',
    )

    return LaunchDescription([use_sim_time_arg, use_odom_arg, container])

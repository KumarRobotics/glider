"""
    Jason Hughes
    June 2025

    launch glider as an
    independent node
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Find package share directory
    glider_share = FindPackageShare('glider')
    glider_share_dir = get_package_share_directory('glider')

    # Path to parameter files
    ros_params_file = PathJoinSubstitution([
        glider_share,
        'config',
        'ros-params.yaml'
    ])

    graph_params_file = PathJoinSubstitution([
        glider_share,
        'config',
        'glider-params.yaml'
    ])

    # create logging directory
    with open(os.path.join(glider_share_dir, "config", "glider-params.yaml")) as f:
        config = yaml.safe_load(f)
    os.makedirs(config["logging"]["directory"], exist_ok=True)

    # Create the glider node
    glider_node = Node(
        package='glider',
        executable='glider_node',
        name='glider_node',
        output='screen',
        parameters=[
            ros_params_file,
            {'path': graph_params_file,
             'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([use_sim_time_arg, glider_node])

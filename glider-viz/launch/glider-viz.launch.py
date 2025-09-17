
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    glider_share = FindPackageShare('glider_viz')

    ros_params_file = PathJoinSubstitution([
        glider_share,
        'map.yaml'
    ])

    glider_viz_node = Node(
        package='glider_viz',
        executable='glider_viz_node',
        name='glider_viz_node',
        output='screen',
        parameters=[
            ros_params_file,
            {"path": glider_share}
        ],
        remappings=[
            ('/odom', '/glider/odom')
        ]
    )

    return LaunchDescription([glider_viz_node])
            

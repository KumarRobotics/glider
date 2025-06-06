import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
    
    # Set global parameters
    set_use_sim_time = SetParameter('use_sim_time', use_sim_time)
    set_use_odom = SetParameter('use_odom', use_odom)
    
    # Create the glider node
    glider_node = Node(
        package='glider',
        executable='glider_ros',
        name='glider_node',
        output='screen',
        parameters=[
            ros_params_file,
            {'path': graph_params_file}
        ],
        remappings=[
            ('/gps', '/ublox/fix'),
            ('/imu', '/imu/data'),
            ('/odom', '/Odometry')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_odom_arg,
        set_use_sim_time,
        set_use_odom,
        glider_node
    ])

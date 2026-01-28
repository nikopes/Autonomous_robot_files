import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Configuration Variables
    use_sim_time = False
    cartographer_config_dir = '/home/niksal'
    configuration_basename = 'my_robot_slam.lua'
    resolution = '0.05'
    publish_period_sec = '1.0'

    return LaunchDescription([
        
        # 2. Start the Main Cartographer Node (SLAM)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
            ],
        ),

        # 3. Start the Occupancy Grid Node (Makes the Map Visible)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec
            ],
        ),
    ])

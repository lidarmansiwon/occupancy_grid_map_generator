import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():
    
    points_topic = LaunchConfiguration('points_topic', default='/filtered_points')

    occupancy_grid_map = Node(
        name='occupancy_grid_map_generator',
        package='occupancy_grid_map_generator',
        executable='occupancy_grid_map_generator',
        remappings=[('/filtered_points', points_topic)],
        parameters=[
            {"min_x_": -20.0},
            {"max_x_": 20.0},
            {"min_y_": -20.0},
            {"max_y_": 20.0},
            {"grid_size_": 0.3},
            {"origin_orientation_x": 0.0},
            {"origin_orientation_y": 0.0},
            {"origin_orientation_z": 0.0},
            {"origin_orientation_w": 1.0},
            {"safety_radius_": 1.0},
            {"boat_position_x_": 0.0},
            {"boat_position_y_": 0.0},
            {"boat_radius_": 2.0},
            {"boat_width_": 0.8},
            {"boat_height_": 2.0}
        ] # Add parameters if needed


    )

    map_tf  = Node(
        name='map_tf',
        package='tf2_ros',
        # namespace=ros_namespace,
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0',
                    '0.707', '0.707', 'grid_map', 'map'],
    )
    
    lidar_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        # namespace=ros_namespace,
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '1.0',
                   '0.0', '0.0', 'grid_map', 'os_lidar'],
    
    )
    
    return LaunchDescription([launch_ros.actions.SetParameter(name='use_sim_time', value=True), lidar_tf, map_tf, occupancy_grid_map])

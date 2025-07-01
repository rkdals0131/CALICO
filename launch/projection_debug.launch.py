#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('calico')
    config_dir = os.path.join(package_dir, '..', '..', '..', '..', 'src', 'calico', 'config')
    default_config = os.path.join(config_dir, 'multi_hungarian_config.yaml')
    
    # Declare launch arguments
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration file'
    )
    
    declare_camera_id_cmd = DeclareLaunchArgument(
        'camera_id',
        default_value='camera_1',
        description='Camera ID to debug (e.g., camera_1, camera_2)'
    )
    
    declare_sync_tolerance_cmd = DeclareLaunchArgument(
        'sync_tolerance',
        default_value='0.1',
        description='Time synchronization tolerance in seconds'
    )
    
    declare_circle_radius_cmd = DeclareLaunchArgument(
        'circle_radius',
        default_value='5',
        description='Radius of projected points in pixels'
    )
    
    # Create the projection debug node
    projection_debug_node = Node(
        package='calico',
        executable='projection_debug_node',
        name='projection_debug_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'camera_id': LaunchConfiguration('camera_id'),
            'sync_tolerance': LaunchConfiguration('sync_tolerance'),
            'circle_radius': LaunchConfiguration('circle_radius'),
        }],
        remappings=[
            # Default remappings, can be overridden
            ('/sorted_cones_time', '/sorted_cones_time'),
            ('/' + LaunchConfiguration('camera_id') + '/image_raw', 
             '/' + LaunchConfiguration('camera_id') + '/image_raw'),
        ]
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_camera_id_cmd)
    ld.add_action(declare_sync_tolerance_cmd)
    ld.add_action(declare_circle_radius_cmd)
    
    # Add nodes
    ld.add_action(projection_debug_node)
    
    return ld
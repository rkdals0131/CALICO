#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """Launch IoU fusion node with configurable parameters."""
    
    # Get package directories
    calico_dir = get_package_share_directory('calico')
    hungarian_dir = get_package_share_directory('hungarian_association')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(calico_dir, 'config', 'iou_fusion_config.yaml'),
        description='Path to the IoU fusion configuration file'
    )
    
    camera_intrinsic_arg = DeclareLaunchArgument(
        'camera_intrinsic',
        default_value=os.path.join(hungarian_dir, 'config', 'camera_intrinsic_calibration.yaml'),
        description='Path to camera intrinsic calibration file'
    )
    
    camera_extrinsic_arg = DeclareLaunchArgument(
        'camera_extrinsic',
        default_value=os.path.join(hungarian_dir, 'config', 'camera_extrinsic_calibration.yaml'),
        description='Path to camera extrinsic calibration file'
    )
    
    enable_debug_viz_arg = DeclareLaunchArgument(
        'enable_debug_viz',
        default_value='true',
        description='Enable debug visualization overlay'
    )
    
    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.1',
        description='Minimum IoU threshold for valid matches'
    )
    
    # IoU Fusion Node
    iou_fusion_node = Node(
        package='calico',
        executable='iou_fusion_node',
        name='iou_fusion_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'camera_intrinsic_file': LaunchConfiguration('camera_intrinsic'),
                'camera_extrinsic_file': LaunchConfiguration('camera_extrinsic'),
                'enable_debug_viz': LaunchConfiguration('enable_debug_viz'),
                'iou_threshold': LaunchConfiguration('iou_threshold'),
            }
        ],
        remappings=[
            # Default remappings - can be overridden
            ('/cone/lidar/box', '/cone/lidar/box'),
            ('/detections', '/detections'),
            ('/usb_cam/image_raw', '/usb_cam/image_raw'),
            ('/fused_cones', '/fused_cones'),
            ('/debug/iou_fusion_overlay', '/debug/iou_fusion_overlay')
        ]
    )
    
    # Optional: RViz for visualization
    rviz_config_file = os.path.join(calico_dir, 'config', 'iou_fusion.rviz')
    
    # Check if RViz config exists
    use_rviz = os.path.exists(rviz_config_file)
    
    ld = LaunchDescription([
        config_file_arg,
        camera_intrinsic_arg,
        camera_extrinsic_arg,
        enable_debug_viz_arg,
        iou_threshold_arg,
        iou_fusion_node
    ])
    
    # Optionally add RViz
    if use_rviz:
        from launch_ros.actions import Node as RvizNode
        rviz_node = RvizNode(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
        ld.add_action(rviz_node)
    
    return ld
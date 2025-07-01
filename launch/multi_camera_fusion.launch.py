from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            '/home/user1/ROS2_Workspace/ros2_ws/src/calico/config',
            'multi_hungarian_config.yaml'
        ),
        description='Path to the configuration file'
    )
    
    # Get configuration file path
    config_file = LaunchConfiguration('config_file')
    
    # Multi-camera fusion node
    multi_camera_fusion_node = Node(
        package='calico',
        executable='multi_camera_fusion_node',
        name='calico_multi_camera_fusion',
        output='screen',
        parameters=[{
            'config_file': config_file
        }],
        remappings=[
            # Add any remappings if needed
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        multi_camera_fusion_node,
    ])
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Default config file path
    default_config_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'config',
        'driveguard_config.yaml'
    )
    
    # Launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the unified configuration YAML file'
    )
    
    # Inference node
    inference_node = Node(
        package='driveguard_ai',
        executable='inference_node',
        name='driveguard_inference',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        inference_node
    ])
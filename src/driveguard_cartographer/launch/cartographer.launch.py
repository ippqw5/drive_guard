from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    pkg_share = FindPackageShare('driveguard_cartographer').find('driveguard_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    map_dir = os.path.join(pkg_share, 'map')
    # 建图时使用 MySlam.lua ，纯定位时使用 Localization.lua
    cartographer_config_file = 'Localization.lua'
    pbstream_file = os.path.join(map_dir, 'new.pbstream')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_robot_localization = LaunchConfiguration('use_robot_localization', default='false')

    ld = LaunchDescription()

    # robot_localization 加强定位
    robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': True}],
            condition=IfCondition(use_robot_localization)
    )
    

    # Cartographer 纯定位节点 (with remapping when use_robot_localization is true)
    cartographer_node_with_remapping = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_localization',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', cartographer_config_file,
            '-load_state_filename', pbstream_file
        ],
        remappings = [
            ('odom', '/odometry/filtered')
        ],
        condition=IfCondition(use_robot_localization)
    )
    
    # Cartographer 纯定位节点 (without remapping when use_robot_localization is false)
    cartographer_node_without_remapping = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_localization',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', cartographer_config_file,
            '-load_state_filename', pbstream_file
        ],
        condition=UnlessCondition(use_robot_localization)
    )

    ld.add_action(robot_localization_node)
    ld.add_action(cartographer_node_with_remapping)
    ld.add_action(cartographer_node_without_remapping)

    return ld
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('driveguard_cartographer').find('driveguard_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    # 建图时使用 MySlam.lua ，纯定位时使用 Localization.lua
    cartographer_config_file = 'Localization.lua'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_dir = os.path.join(pkg_share, 'config')+"/cartographer.rviz"
    pbstream_file = os.path.join(config_dir, 'my.pbstream')

    return LaunchDescription([
        # Cartographer 纯定位节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_localization',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', cartographer_config_file,
                # 建图时不使用，纯定位时使用
                '-load_state_filename', pbstream_file
            ]
        ),
        # Cartographer 地图发布节点（替换 map_server）
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_map_server',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
        ),
    ])

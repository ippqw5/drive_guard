from launch import LaunchDescription
from launch.actions import Node, IfCondition
from launch.substitutions import LaunchConfiguration, NotEqualsSubstitution

def generate_launch_description():
    # 定义启动配置
    use_respawn = LaunchConfiguration('use_respawn', default='false')
    log_level = LaunchConfiguration('log_level', default='info')
    map_yaml_file = '/home/driveguard/Desktop/ROS2_NAV2_CARTO/ROS2_Nav2/src/fishbot_navigation2/maps/room.yaml'

    # 配置参数
    configured_params = {}

    # 话题重映射
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        Node(
            condition=IfCondition(
                NotEqualsSubstitution(map_yaml_file, '')
            ),
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, {'yaml_filename': map_yaml_file}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        )
    ])
    
# ros2 launch launch_map_server.py map:=/path/to/your/map.yaml
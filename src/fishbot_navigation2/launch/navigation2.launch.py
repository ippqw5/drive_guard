import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():


    # 定义启动配置
    use_respawn = launch.substitutions.LaunchConfiguration('use_respawn', default='false')
    log_level = launch.substitutions.LaunchConfiguration('log_level', default='info')

    # 配置参数
    configured_params = {}

    # 话题重映射
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]










    # 获取与拼接默认路径
    fishbot_navigation2_dir = get_package_share_directory(
        'fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))
    my_launch_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'launch', 'my.launch.py'))

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [my_launch_path]),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                # 'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        # launch_ros.actions.Node(
        #     # condition=IfCondition(
        #     #     NotEqualsSubstitution(map_yaml_file, '')
        #     # ),
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[configured_params, {'yaml_filename': map_yaml_path}],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     remappings=remappings),

    ])
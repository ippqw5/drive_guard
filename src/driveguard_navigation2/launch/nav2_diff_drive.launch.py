import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取与拼接默认路径
    driveguard_navigation2_dir = get_package_share_directory(
        'driveguard_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        driveguard_navigation2_dir, 'rviz', 'driveguard_default_view.rviz')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    map_yaml_file = os.path.join(driveguard_navigation2_dir, 'maps', 'map.yaml')

    lifecycle_nodes = ['map_server']

    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(driveguard_navigation2_dir, 'config', 'diff_drive_params.yaml'))
    use_arbitrator = launch.substitutions.LaunchConfiguration(
        'use_arbitrator', default='false')
    
    # 如果使用[]嵌套对象需要解析位为字符串
    # my_launch_path = launch.substitutions.LaunchConfiguration(
    #     'params_file', default=os.path.join(driveguard_navigation2_dir, 'launch', 'my.launch.py'))
    # context = LaunchContext()
    # my_launch_path_str = my_launch_path.perform(context)   
    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),
        launch.actions.DeclareLaunchArgument('use_arbitrator', default_value='false',
                                             description='Use arbitrator for navigation if true'),

        launch.actions.IncludeLaunchDescription(
            # my_launch_path = /home/driveguard/Desktop/drive_guard/install/driveguard_navigation2/share/driveguard_navigation2/launch/my.launch.py
            # 上这个不行，需要用下边这个路径
            PythonLaunchDescriptionSource(
                os.path.join(driveguard_navigation2_dir, 'launch', 'my.launch.py')),
                # 如果使用[]嵌套对象需要解析位为字符串
                # [my_launch_path_str]),

            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'use_arbitrator': use_arbitrator
                }.items(),
        ),

        launch_ros.actions.Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            ),

        launch_ros.actions.Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
            ),

        # Cartographer 地图发布节点（替换 map_server）
        # launch_ros.actions.Node(
        #     package='cartographer_ros',
        #     executable='cartographer_occupancy_grid_node',
        #     name='cartographer_occupancy_grid_node',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        # ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",   -- 机器人主坐标系
  published_frame = "odom",  -- 机器人发布的坐标（map->odom）
  odom_frame = "null",       -- 没用到
  provide_odom_frame = false,     -- 不提供里程计数据
  publish_frame_projected_to_2d = true,
  use_odometry = true,           -- 使用外部里程计数据
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,            -- 只用一个雷达
  num_multi_echo_laser_scans = 0,    -- 不使用多波雷达
  num_subdivisions_per_laser_scan = 1,  -- 1/1=1不分割（涉及多回波）
  num_point_clouds = 0,            -- 3D点云
  lookup_transform_timeout_sec = 0.2,  --tf变换的超时时间
  submap_publish_period_sec = 0.3,    -- 子地图发布间隔（s）
  pose_publish_period_sec = 5e-3,     -- 机器人发布位姿频率（s）
  trajectory_publish_period_sec = 30e-3,  -- 轨迹可视化发布间隔（s）
  rangefinder_sampling_ratio = 1.,  -- 激光雷达数据采样率（0~1）
  odometry_sampling_ratio = 1.,    -- 里程计数据采样率
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,               -- IMU数据采样率
  landmarks_sampling_ratio = 1.,  -- 标志物数据采样率
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- 2d建图

TRAJECTORY_BUILDER_2D.min_range = 0.1 --激光的最近距离（考虑机器人半径）
TRAJECTORY_BUILDER_2D.max_range = 15  --激光的最远距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.  -- 传感器数据超出有效范围最大值
TRAJECTORY_BUILDER_2D.use_imu_data = true    -- 使用IMU时，IMU数据的坐标系（cartographer默认认为是odom.child）要和 tracking_frame 一致(imu_link和base_footprint最好一致)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 使用实时回环检测来进行前端的匹配
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 1改为0.1，提高对运动的敏感度

POSE_GRAPH.constraint_builder.min_score = 0.65 -- 0.55改成0.65，fast csm的最低分数，高于此分数才进行优化
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- 0.6改成0.7，全局定位最小分数，低于此分数认为全局定位不准确
POSE_GRAPH.optimize_every_n_nodes = 10  -- 设置0可关闭全局优化，-- 每10个有效帧一个子图，子图构建完成要闭环检测一次，该数越小，闭环检测越频繁

return options

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  start_trajectory_with_default_topics = false,
  use_odometry = false,  -- 禁用里程计（根据需求）
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.1,
}

-- 禁用 SLAM（仅定位）
MAP_BUILDER.use_trajectory_builder_2d = false  -- 关闭 2D 建图
MAP_BUILDER.use_trajectory_builder_3d = false  -- 关闭 3D 建图

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 0  -- 禁用扫描匹配
TRAJECTORY_BUILDER_2D.min_range = 0.1  -- 激光雷达参数
TRAJECTORY_BUILDER_2D.max_range = 30.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 30.0

return options

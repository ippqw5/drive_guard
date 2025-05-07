include "MySlam.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,  -- 最大保存子图数，纯定位模式通过子图进行定位
}

-- MAP_BUILDER.num_background_threads = 12
POSE_GRAPH.constraint_builder.sampling_ratio = POSE_GRAPH.constraint_builder.sampling_ratio  -- （默认0.3）无所谓
POSE_GRAPH.global_sampling_ratio = 10* POSE_GRAPH.global_sampling_ratio  -- （提高回环检测的频率与准确度，会带来显著的计算负担，默认0.003）
-- POSE_GRAPH.max_num_final_iterations = 1  （默认200）

return options

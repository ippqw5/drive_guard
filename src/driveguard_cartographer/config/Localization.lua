include "MySlam.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,  -- 最大保存子图数，纯定位模式通过子图进行定位
}
POSE_GRAPH.optimize_every_n_nodes = 20  -- 每20个有效帧一个子图，子图构建完成要闭环检测一次，该数越小，闭环检测越频繁

 
return options

include "MySlam.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,  -- 最大保存子图数，纯定位模式通过子图进行定位
}

return options

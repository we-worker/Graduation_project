-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "imu_link",
--   published_frame = "imu_link",
--   odom_frame = "odom",
--   provide_odom_frame = true,
--   publish_frame_projected_to_2d = false,
--   use_pose_extrapolator = true,
--   use_odometry = false,
--   use_nav_sat = false,
--   use_landmarks = false,
--   num_laser_scans = 0,
--   num_multi_echo_laser_scans = 1,
--   num_subdivisions_per_laser_scan = 10,
--   num_point_clouds = 0,
--   lookup_transform_timeout_sec = 0.2,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 1.,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,
-- }

-- MAP_BUILDER.use_trajectory_builder_2d = true
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

-- return options



include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",  --如果有imu填imu,没有imu则用imu_link
  published_frame = "scanner_link",   --有odom一般用odom，没有odom一般用imu_link
  odom_frame = "odom",
  provide_odom_frame = true,  --底盘提供了里程计，这里不使用算法提供的里程计；如果没有底盘提供，则可以用cartographer提供的里程计，这里摄制成true
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,   --使能位姿
  use_odometry = false,  --是否使用里程计（底盘提供）
  use_nav_sat = false, 
  use_landmarks = false,
  num_laser_scans = 1,  -- 激光雷达数量
  num_multi_echo_laser_scans = 0,  --google提供的激光雷达，这里用不到，为0
  num_subdivisions_per_laser_scan = 1,  --每扫描一帧作为一帧，原始值为10帧为一帧
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true   --对应map_builder.lua文件，这里设置成true.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 4  --每4帧取一帧，防止雷达帧率太高耗费计算资源，可以减少计算量
TRAJECTORY_BUILDER_2D.use_imu_data = true

return options


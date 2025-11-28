include "map_builder.lua"                   --包含的cartographer里的lua文件
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "base_footprint",             -- 将所有传感器数据转换到这个坐标系下，如果有imu的，就写imu的link，如果没有，就写base_link或者footprint，
                                            -- 因为cartographer会将所有传感器进行坐标变换到tracking_fram坐标系下，
                                            -- 每个传感器的频率不一样，imu频率远高于雷达的频率，这样做可以减少计算量
  published_frame = "base_footprint",       -- tf: map -> footprint 自己bag的tf最上面的坐标系的名字
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = false,                -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上，没啥用
  --use_pose_extrapolator = false,            -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿，前端计算的位姿更准

  use_odometry = true,                      -- 是否使用里程计,如果使用要求一定要有odom的tf *
  use_nav_sat = false,                      -- 是否使用gps *topic形式订阅，不可订阅多个里程计/gps/landmark，要注意做重映射
  use_landmarks = false,                    -- 是否使用landmark *
  num_laser_scans = 1,                      -- 是否使用单线激光数据，可订阅多个
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用点云数据

  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率
  odometry_sampling_ratio = 1.,             -- 设成0.1即来10帧用1帧
  fixed_frame_pose_sampling_ratio = 1.,     -- 某个传感器不准，可以降低其使用频率
  imu_sampling_ratio = 1.,                  -- 如若不准，一般直接弃用即可
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
--  TEST 
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
-- MAP_BUILDER.num_background_threads = 4

--  TEST
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
-- POSE_GRAPH.global_sampling_ratio = 0.001
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.001

-- POSE_GRAPH.optimize_every_n_nodes = 50

return options

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Pfad zur Parameter-Datei (params.yaml)
    pkg_share = get_package_share_directory('radar_graph_slam')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'radar_graph_slam.rviz')

    ld = LaunchDescription()

    # --- LAUNCH ARGUMENTS ---
    arguments = [
        ('enable_floor_detection', 'true'),
        ('enable_barometer', 'false'),
        ('barometer_edge_type', '1'),
        ('enable_gps', 'false'),
        ('enable_dynamic_object_removal', 'false'),
        ('enable_frontend_ego_vel', 'false'),
        ('enable_preintegration', 'false'),
        ('keyframe_delta_trans_front_end', '0.2'),
        ('keyframe_delta_trans_back_end', '2.0'),
        ('keyframe_delta_angle', '0.15'),
        ('enable_transform_thresholding', 'true'),
        ('enable_loop_closure', 'false'),
        ('registration_method', 'FAST_APDGICP'),
        ('reg_resolution', '0.2'),
        ('dist_var', '0.86'),
        ('azimuth_var', '0.5'),
        ('elevation_var', '1.0'),
        ('enable_robot_odometry_init_guess', 'false'),
        ('points_topic', '/filtered_points'),
    ]

    for arg_name, default_value in arguments:
        ld.add_action(DeclareLaunchArgument(arg_name, default_value=default_value))

    # --- KNOTEN 1: Static Transform Publisher ---
    radar_to_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imaging_radar1'],
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(radar_to_base_node)

    # --- KNOTEN 2: Radar Preprocessing Node ---
    preprocessing_node = Node(
        package='radar_graph_slam',
        executable='preprocessing_node',
        name='radar_preprocessing_node',
        output='screen',
        remappings=[
            ('radar_points', '/imaging_radar1'),
            ('radar_twist', '/imaging_radar1_twist')
        ],
        parameters=[
            params_file,
            {
                'use_sim_time': True,
                'use_distance_filter': True,
                'distance_near_thresh': 1.0,
                'distance_far_thresh': 100.0,
                'z_low_thresh': -2.0,
                'z_high_thresh': 5.0,
                'downsample_method': 'VOXELGRID',
                'downsample_resolution': 0.05,
                'outlier_removal_method': 'RADIUS',
                'statistical_mean_k': 30,
                'statistical_stddev': 0.2,
                'radius_radius': 0.5,
                'radius_min_neighbors': 1,
                'bilateral_sigma_s': 5,
                'bilateral_sigma_r': 0.03,
                'power_threshold': -20.0,
                'enable_dynamic_object_removal': LaunchConfiguration('enable_dynamic_object_removal'),
                'gt_file_location': '/home/zhuge/datasets/Radar/gt_carpark1.txt'
            }
        ]
    )
    ld.add_action(preprocessing_node)

    # --- KNOTEN 3: Scan Matching Odometry Node ---
    scan_matching_odometry_node = Node(
        package='radar_graph_slam',
        executable='scan_matching_odometry_node',
        name='scan_matching_odometry_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': True,
                'keyframe_delta_trans': LaunchConfiguration('keyframe_delta_trans_front_end'),
                'keyframe_delta_angle': LaunchConfiguration('keyframe_delta_angle'),
                'enable_robot_odometry_init_guess': LaunchConfiguration('enable_robot_odometry_init_guess'),
                'enable_transform_thresholding': LaunchConfiguration('enable_transform_thresholding'),
                'enable_imu_thresholding': False,
                'max_acceptable_trans': 3.0,
                'max_acceptable_angle': 5.0,
                'max_diff_trans': 0.3,
                'max_diff_angle': 0.8,
                'max_egovel_cum': 1.0,
                'downsample_method': 'VOXELGRID',
                'downsample_resolution': 0.3,
                'registration_method': LaunchConfiguration('registration_method'),
                'dist_var': LaunchConfiguration('dist_var'),
                'azimuth_var': LaunchConfiguration('azimuth_var'),
                'elevation_var': LaunchConfiguration('elevation_var'),
                'reg_num_threads': 0,
                'reg_transformation_epsilon': 0.1,
                'reg_maximum_iterations': 128,
                'reg_max_correspondence_distance': 2.0,
                'reg_max_optimizer_iterations': 20,
                'reg_use_reciprocal_correspondences': False,
                'reg_correspondence_randomness': 20,
                'reg_resolution': LaunchConfiguration('reg_resolution'),
                'reg_nn_search_method': 'DIRECT7',
                'use_ego_vel': True,
                'max_submap_frames': 5,
                'enable_scan_to_map': False,
                'enable_imu_fusion': False,
                'imu_debug_out': True,
                'imu_fusion_ratio': 0.1,
                'odom_topic': '/odom'
            }
        ]
    )
    ld.add_action(scan_matching_odometry_node)

    # --- KNOTEN 4: Radar Graph SLAM Node ---
    radar_graph_slam_node = Node(
        package='radar_graph_slam',
        executable='radar_graph_slam_node',
        name='radar_graph_slam_node',
        prefix=['gdb -ex run --args'],
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': True,
                'g2o_solver_type': 'lm_var_csparse',
                'g2o_solver_num_iterations': 512,
                'enable_barometer': LaunchConfiguration('enable_barometer'),
                'enable_gps': LaunchConfiguration('enable_gps'),
                'keyframe_delta_trans': LaunchConfiguration('keyframe_delta_trans_back_end'),
                'keyframe_delta_angle': LaunchConfiguration('keyframe_delta_angle'),
                'fix_first_node': True,
                'fix_first_node_stddev': '10 10 10 1 1 1', 
                'fix_first_node_adaptive': True,
                'enable_loop_closure': LaunchConfiguration('enable_loop_closure'),
                'enable_pf': True,
                'enable_odom_check': True,
                'distance_thresh': 10.0,
                'accum_distance_thresh': 50.0,
                'min_loop_interval_dist': 10.0,
                'distance_from_last_edge_thresh': 10.0,
                'max_baro_difference': 2.0,
                'max_yaw_difference': 20.0,
                'sc_dist_thresh': 0.5,
                'sc_azimuth_range': 56.5,
                'historyKeyframeFitnessScore': 6.0,
                'odom_check_trans_thresh': 0.3,
                'odom_check_rot_thresh': 0.05,
                'pairwise_check_trans_thresh': 1.5,
                'pairwise_check_rot_thresh': 0.2,
                'registration_method': LaunchConfiguration('registration_method'),
                'reg_num_threads': 0,
                'reg_transformation_epsilon': 0.1,
                'reg_maximum_iterations': 128,
                'reg_max_correspondence_distance': 2.0,
                'reg_max_optimizer_iterations': 20,  
                'reg_use_reciprocal_correspondences': False,
                'reg_correspondence_randomness': 20,
                'reg_resolution': LaunchConfiguration('reg_resolution'),
                'reg_nn_search_method': 'DIRECT7',
                'barometer_edge_type': LaunchConfiguration('barometer_edge_type'),
                'barometer_edge_robust_kernel': 'Huber',
                'barometer_edge_robust_kernel_size': 1.0,
                'barometer_edge_stddev': 0.47,
                'gps_edge_robust_kernel': 'Huber',
                'gps_edge_robust_kernel_size': 1.0,
                'gps_edge_stddev_xy': 5.0,
                'gps_edge_stddev_z': 5.0,
                'max_gps_edge_stddev_xy': 1.5,
                'max_gps_edge_stddev_z': 3.0,
                'gps_edge_intervals': 15,
                'dataset_name': 'loop2',
                'enable_preintegration': LaunchConfiguration('enable_preintegration'),
                'use_egovel_preinteg_trans': False,
                'preinteg_orient_stddev': 1.0,
                'preinteg_trans_stddev': 5.0,
                'odometry_edge_robust_kernel': 'NONE',
                'odometry_edge_robust_kernel_size': 1.0,
                'loop_closure_edge_robust_kernel': 'Huber',
                'loop_closure_edge_robust_kernel_size': 1.0,
                'use_const_inf_matrix': False,
                'const_stddev_x': 0.5,
                'const_stddev_q': 0.1,
                'var_gain_a': 20.0,
                'min_stddev_x': 0.1,
                'max_stddev_x': 5.0,
                'min_stddev_q': 0.05,
                'max_stddev_q': 0.2,
                'graph_update_interval': 2.0,
                'map_cloud_update_interval': 6.0,
                'map_cloud_resolution': 0.05,
                'show_sphere': False,
                'points_topic': LaunchConfiguration('points_topic'),
                'odom_topic': '/odom'
            }
        ]
    )
    ld.add_action(radar_graph_slam_node)

    # --- KNOTEN 5: RViz2 ---
    rviz_config_dir = os.path.join(get_package_share_directory('radar_graph_slam'),'my.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_slam',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        respawn=True
    )
    ld.add_action(rviz_node)

    return ld
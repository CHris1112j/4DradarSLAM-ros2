cmake_minimum_required(VERSION 3.5)
project(radar_graph_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(barometer_bmp388 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filtering)
find_package(G2O QUIET)
if(NOT G2O_FOUND)
  find_path(G2O_INCLUDE_DIR NAMES g2o/core/base_vertex.h)
  if(G2O_INCLUDE_DIR)
    message(STATUS "G2O found in: ${G2O_INCLUDE_DIR}")
    set(G2O_FOUND TRUE)
  else()
    message(FATAL_ERROR "G2O not found! Bitte mit 'sudo apt install ros-humble-libg2o' installieren.")
  endif()
endif()
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloorCoeffs.msg"
  "msg/ScanMatchingStatus.msg"
  "srv/SaveMap.srv"
  "srv/DumpGraph.srv"
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs nmea_msgs geometry_msgs
)

add_library(preprocessing_nodelet SHARED
  apps/preprocessing_nodelet.cpp
  src/radar_ego_velocity_estimator.cpp
)

add_library(scan_matching_odometry_nodelet SHARED
  apps/scan_matching_odometry_nodelet.cpp 
  src/radar_graph_slam/registrations.cpp
  src/radar_graph_slam/keyframe.cpp
  src/radar_graph_slam/information_matrix_calculator.cpp
)

add_library(radar_graph_slam_nodelet SHARED
  apps/radar_graph_slam_nodelet.cpp 
  src/radar_graph_slam/graph_slam.cpp 
  src/radar_graph_slam/loop_detector.cpp
  src/radar_graph_slam/Scancontext.cpp 
  src/radar_graph_slam/keyframe.cpp
  src/radar_graph_slam/map_cloud_generator.cpp
  src/radar_graph_slam/registrations.cpp
  src/radar_graph_slam/information_matrix_calculator.cpp
  src/g2o/robust_kernel_io.cpp
)

rclcpp_components_register_nodes(preprocessing_nodelet "radar_graph_slam::PreprocessingNodelet")
rclcpp_components_register_node(
  preprocessing_nodelet
  PLUGIN "radar_graph_slam::PreprocessingNodelet"
  EXECUTABLE preprocessing_node
)

rclcpp_components_register_nodes(scan_matching_odometry_nodelet "radar_graph_slam::ScanMatchingOdometryNodelet")
rclcpp_components_register_node(
  scan_matching_odometry_nodelet
  PLUGIN "radar_graph_slam::ScanMatchingOdometryNodelet"
  EXECUTABLE scan_matching_odometry_node
)

rclcpp_components_register_nodes(radar_graph_slam_nodelet "radar_graph_slam::RadarGraphSlamNodelet")
rclcpp_components_register_node(
  radar_graph_slam_nodelet
  PLUGIN "radar_graph_slam::RadarGraphSlamNodelet"
  EXECUTABLE radar_graph_slam_node
)

set(ALL_TARGETS 
  preprocessing_nodelet 
  scan_matching_odometry_nodelet 
  radar_graph_slam_nodelet
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

set(G2O_LIBS
  /opt/ros/humble/lib/x86_64-linux-gnu/libg2o_core.so
  /opt/ros/humble/lib/x86_64-linux-gnu/libg2o_stuff.so
  /opt/ros/humble/lib/x86_64-linux-gnu/libg2o_types_slam3d.so
  /opt/ros/humble/lib/x86_64-linux-gnu/libg2o_types_slam3d_addons.so
  /opt/ros/humble/lib/x86_64-linux-gnu/libg2o_solver_pcg.so
  /opt/ros/humble/lib/x86_64-linux-gnu/libg2o_solver_csparse.so
  /opt/ros/humble/lib/x86_64-linux-gnu/libg2o_solver_cholmod.so
)

foreach(target ${ALL_TARGETS})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    /home/mo/radar_ws/install/fast_gicp/include
  )

  ament_target_dependencies(${target}
    rclcpp pluginlib std_msgs tf2_eigen tf2_geometry_msgs
    tf2_sensor_msgs sensor_msgs 
    ndt_omp geometry_msgs nav_msgs nmea_msgs geodesy pcl_ros 
    pcl_conversions cv_bridge image_transport interactive_markers
    barometer_bmp388
  )

link_directories(
    /opt/ros/humble/lib/x86_64-linux-gnu
    /home/mo/radar_ws/install/fast_gicp/lib
  )

link_directories(/opt/ros/humble/lib/x86_64-linux-gnu)
  target_link_libraries(${target} 
    ${cpp_typesupport_target}
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    fast_gicp
    ${G2O_LIBS}
    gtsam
  )
endforeach()

install(TARGETS ${ALL_TARGETS}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  src/${PROJECT_NAME}/bag_player.py
  src/${PROJECT_NAME}/ford2bag.py
  src/${PROJECT_NAME}/map2odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${ALL_TARGETS})
ament_export_dependencies(
  ament_cmake_ros
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
  geodesy
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  PCL
  OpenCV
  G2O
  GTSAM
  pcl_conversions
  ndt_omp
)
 
ament_package()
cmake_minimum_required(VERSION 3.14)
project(ros2_obstacle_detection_on_occupancy_map)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/ros2_obstacle_detection_on_occupancy_map/ros2_obstacle_detection_on_occupancy_map.hpp
  include/ros2_obstacle_detection_on_occupancy_map/visibility_control.hpp
  src/ros2_obstacle_detection_on_occupancy_map.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/ros2_obstacle_detection_on_occupancy_map/ros2_obstacle_detection_on_occupancy_map_node.hpp
  src/ros2_obstacle_detection_on_occupancy_map_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "ros2_obstacle_detection_on_occupancy_map::Ros2ObstacleDetectionOnOccupancyMapNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_ros2_obstacle_detection_on_occupancy_map
    test/test_ros2_obstacle_detection_on_occupancy_map.cpp
  )
  ament_target_dependencies(test_ros2_obstacle_detection_on_occupancy_map ${TEST_SOURCES})
  target_link_libraries(test_ros2_obstacle_detection_on_occupancy_map ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

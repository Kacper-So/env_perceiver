cmake_minimum_required(VERSION 3.14)
project(env_perceiver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)

# Add executable
add_executable(env_perceiver src/EnvPerceiver.cpp)

# Include directories for the target
target_include_directories(env_perceiver PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${pcl_ros_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${autoware_auto_planning_msgs_INCLUDE_DIRS}
)

# Link executable with PCL libraries
target_link_libraries(env_perceiver
    ${PCL_LIBRARIES}
)

# Link dependencies
ament_target_dependencies(env_perceiver
    autoware_auto_planning_msgs
    rclcpp
    nav_msgs
    sensor_msgs
    pcl_conversions
    pcl_ros
)

# Install executable
install(TARGETS env_perceiver
        DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()

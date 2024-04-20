// Copyright 2024 kacper-so
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP__ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_NODE_HPP_
#define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP__ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "ros2_obstacle_detection_on_occupancy_map/ros2_obstacle_detection_on_occupancy_map.hpp"

namespace ros2_obstacle_detection_on_occupancy_map
{
using Ros2ObstacleDetectionOnOccupancyMapPtr = std::unique_ptr<ros2_obstacle_detection_on_occupancy_map::Ros2ObstacleDetectionOnOccupancyMap>;

class ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_PUBLIC Ros2ObstacleDetectionOnOccupancyMapNode : public rclcpp::Node
{
public:
  explicit Ros2ObstacleDetectionOnOccupancyMapNode(const rclcpp::NodeOptions & options);

private:
  Ros2ObstacleDetectionOnOccupancyMapPtr ros2_obstacle_detection_on_occupancy_map_{nullptr};
  void occupancyMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void updateOccupancyMap();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_publisher_;

  nav_msgs::msg::OccupancyGrid occupancy_map_;
};
}  // namespace ros2_obstacle_detection_on_occupancy_map

#endif  // ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP__ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_NODE_HPP_

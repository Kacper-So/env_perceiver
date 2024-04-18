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
  int64_t param_name_{123};
};
}  // namespace ros2_obstacle_detection_on_occupancy_map

#endif  // ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP__ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_NODE_HPP_

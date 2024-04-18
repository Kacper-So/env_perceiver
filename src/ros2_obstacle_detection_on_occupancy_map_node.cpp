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

#include "ros2_obstacle_detection_on_occupancy_map/ros2_obstacle_detection_on_occupancy_map_node.hpp"

namespace ros2_obstacle_detection_on_occupancy_map
{

Ros2ObstacleDetectionOnOccupancyMapNode::Ros2ObstacleDetectionOnOccupancyMapNode(const rclcpp::NodeOptions & options)
:  Node("ros2_obstacle_detection_on_occupancy_map", options)
{
  ros2_obstacle_detection_on_occupancy_map_ = std::make_unique<ros2_obstacle_detection_on_occupancy_map::Ros2ObstacleDetectionOnOccupancyMap>();
  param_name_ = this->declare_parameter("param_name", 456);
  ros2_obstacle_detection_on_occupancy_map_->foo(param_name_);
}

}  // namespace ros2_obstacle_detection_on_occupancy_map

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_obstacle_detection_on_occupancy_map::Ros2ObstacleDetectionOnOccupancyMapNode)

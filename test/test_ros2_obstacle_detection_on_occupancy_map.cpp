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

#include "gtest/gtest.h"
#include "ros2_obstacle_detection_on_occupancy_map/ros2_obstacle_detection_on_occupancy_map.hpp"

TEST(TestRos2ObstacleDetectionOnOccupancyMap, TestHello) {
  std::unique_ptr<ros2_obstacle_detection_on_occupancy_map::Ros2ObstacleDetectionOnOccupancyMap> ros2_obstacle_detection_on_occupancy_map_ =
    std::make_unique<ros2_obstacle_detection_on_occupancy_map::Ros2ObstacleDetectionOnOccupancyMap>();
  auto result = ros2_obstacle_detection_on_occupancy_map_->foo(999);
  EXPECT_EQ(result, 999);
}

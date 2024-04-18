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

#ifndef ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP__VISIBILITY_CONTROL_HPP_
#define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_BUILDING_DLL) || defined(ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_EXPORTS)
    #define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_PUBLIC __declspec(dllexport)
    #define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_LOCAL
  #else  // defined(ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_BUILDING_DLL) || defined(ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_EXPORTS)
    #define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_PUBLIC __declspec(dllimport)
    #define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_LOCAL
  #endif  // defined(ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_BUILDING_DLL) || defined(ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_EXPORTS)
#elif defined(__linux__)
  #define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_PUBLIC __attribute__((visibility("default")))
  #define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_PUBLIC __attribute__((visibility("default")))
  #define ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // ROS2_OBSTACLE_DETECTION_ON_OCCUPANCY_MAP__VISIBILITY_CONTROL_HPP_

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
    // Subscribe to occupancy map topic
    occupancy_map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&OccupancyMapUpdater::occupancyMapCallback, this, std::placeholders::_1));

    // Subscribe to LiDAR topic
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar_topic", 10, std::bind(&OccupancyMapUpdater::lidarCallback, this, std::placeholders::_1));

    // Subscribe to odometry topic
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OccupancyMapUpdater::odometryCallback, this, std::placeholders::_1));

    // Publish updated occupancy map
    occupancy_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/updated_map", 10);
}


// Callback function for occupancy map
Ros2ObstacleDetectionOnOccupancyMapNode::occupancyMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Update the local copy of the occupancy map
    occupancy_map_ = *msg;
}

// Callback function for LiDAR data
Ros2ObstacleDetectionOnOccupancyMapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Convert LiDAR data to Cartesian coordinates (x, y)
    std::vector<geometry_msgs::msg::Point> points;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        // Check for valid range
        if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
        {
            continue;
        }

        // Convert polar coordinates to Cartesian coordinates
        double angle = msg->angle_min + i * msg->angle_increment;
        double x = msg->ranges[i] * std::cos(angle);
        double y = msg->ranges[i] * std::sin(angle);

        // Create a point
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;

        points.push_back(point);
    }

    // Perform clustering algorithm (e.g., DBSCAN, K-means) on the points
    // Update occupancy map based on clusters
    updateOccupancyMap();
}

// Callback function for odometry
Ros2ObstacleDetectionOnOccupancyMapNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Process odometry data if needed
    // For example, you can extract the robot's pose (position and orientation) from the odometry message
    // You can use this pose information for localization or to assist in updating the occupancy map
    geometry_msgs::msg::Pose pose = msg->pose.pose;

    // Example: Print robot's position
    RCLCPP_INFO(this->get_logger(), "Robot's position: [x=%f, y=%f]", pose.position.x, pose.position.y);

    // Example: Print robot's orientation (quaternion)
    RCLCPP_INFO(this->get_logger(), "Robot's orientation: [x=%f, y=%f, z=%f, w=%f]",
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    // You can further process the odometry data here based on your application requirements
}

// Function to update occupancy map based on clusters
Ros2ObstacleDetectionOnOccupancyMapNode::updateOccupancyMap()
{
    // Assuming occupancy_map_ is a member variable representing the current occupancy map

    // Iterate through the clusters and update the occupancy map
    for (const auto& cluster : clusters)
    {
        // Convert cluster points to grid coordinates
        for (const auto& point : cluster.points)
        {
            // Convert Cartesian coordinates to grid coordinates
            int grid_x = (int)((point.x - occupancy_map_.info.origin.position.x) / occupancy_map_.info.resolution);
            int grid_y = (int)((point.y - occupancy_map_.info.origin.position.y) / occupancy_map_.info.resolution);

            // Update the occupancy probability of the corresponding grid cell
            if (grid_x >= 0 && grid_x < occupancy_map_.info.width && grid_y >= 0 && grid_y < occupancy_map_.info.height)
            {
                // Assuming binary occupancy (occupied or free), you might need more complex updating logic for probabilistic occupancy
                occupancy_map_.data[grid_y * occupancy_map_.info.width + grid_x] = 100; // Mark cell as occupied
            }
        }
    }

    // Publish the updated occupancy map
    occupancy_map_publisher_->publish(occupancy_map_);
}


}  // namespace ros2_obstacle_detection_on_occupancy_map

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_obstacle_detection_on_occupancy_map::Ros2ObstacleDetectionOnOccupancyMapNode)

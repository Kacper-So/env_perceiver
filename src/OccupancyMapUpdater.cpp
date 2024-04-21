#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

class OccupancyMapUpdater : public rclcpp::Node {
public:
    OccupancyMapUpdater() : Node("occupancy_map_updater") {
        occupancy_map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&OccupancyMapUpdater::occupancyMapCallback, this, std::placeholders::_1));
        
        rclcpp::QoS qos(rclcpp::KeepLast(5)); // Example QoS settings
        qos.best_effort(); // Set the reliability to best effort
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/sensing/lidar/scan", qos, std::bind(&OccupancyMapUpdater::lidarCallback, this, std::placeholders::_1));

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state", 10, std::bind(&OccupancyMapUpdater::odometryCallback, this, std::placeholders::_1));

        updated_occupancy_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/updated_map", 10);

        epsilon_ = 0.2;  // Adjust epsilon according to your LiDAR sensor's resolution
        min_points_ = 5;  // Adjust min_points as needed
        increase_probability_ = 5; // Adjust probability increments as needed
        decrease_probability_ = 5; // Adjust probability decrements as needed
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr OG;
    nav_msgs::msg::Odometry::SharedPtr curr_odometry;
    double epsilon_;
    int min_points_;
    int increase_probability_;
    int decrease_probability_;
    float map_origin_x_;
    float map_origin_y_;
    float map_resolution_;
    float map_width_;

    void occupancyMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_msg) {
        RCLCPP_INFO(this->get_logger(), "Received occupancy map");
        OG = occupancy_map_msg;
        map_origin_x_ = occupancy_map_msg->info.origin.position.x;
        map_origin_y_ = occupancy_map_msg->info.origin.position.y;
        map_resolution_ = occupancy_map_msg->info.resolution;
        map_width_ = occupancy_map_msg->info.width;
        RCLCPP_INFO(this->get_logger(), "Map Origin: (%f, %f)", map_origin_x_, map_origin_y_);
        RCLCPP_INFO(this->get_logger(), "Map Resolution: %f", map_resolution_);
        RCLCPP_INFO(this->get_logger(), "Map Width: %f", map_width_);
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg) {
        RCLCPP_INFO(this->get_logger(), "Received LiDAR data");

        if (!OG || !curr_odometry) {
            RCLCPP_INFO(this->get_logger(), "Occupancy grid or odometry not received yet");
            return;
        }

        // Process LiDAR data
        std::vector<std::pair<double, double>> points;
        for (size_t i = 0; i < lidar_msg->ranges.size(); ++i) {
            double angle = lidar_msg->angle_min + i * lidar_msg->angle_increment;
            double range = lidar_msg->ranges[i];
            if (range < lidar_msg->range_min || range > lidar_msg->range_max)
                continue; // Skip invalid range
            double x = range * cos(angle);
            double y = range * sin(angle);
            points.push_back({x, y});
        }
        RCLCPP_INFO(this->get_logger(), "Number of LiDAR Points: %zu", points.size());

        // Transform LiDAR points to map frame
        std::vector<std::pair<double, double>> transformed_points = transformLidarPoints(points, *curr_odometry);
        RCLCPP_INFO(this->get_logger(), "Number of Transformed Points: %zu", transformed_points.size());

        // Update occupancy grid with LiDAR data
        updateOccupancyGrid(transformed_points);
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");

        curr_odometry = odometry_msg;
    }

    void updateOccupancyGrid(const std::vector<std::pair<double, double>>& points) {
    // Set to store grid cells updated by Lidar scan
    std::unordered_set<int> updated_cells;

    // Update occupancy grid with LiDAR points
    for (const auto& point : points) {
        // Convert point to grid cell coordinates
        int grid_x = static_cast<int>((point.first - map_origin_x_) / map_resolution_);
        int grid_y = static_cast<int>((point.second - map_origin_y_) / map_resolution_);

        if (grid_x >= 0 && grid_x < OG->info.width && grid_y >= 0 && grid_y < OG->info.height) {
            // Update cell probability
            int index = grid_y * OG->info.width + grid_x;
            OG->data[index] += increase_probability_; // Increase probability (up to 100)
            OG->data[index] = std::min(static_cast<int>(OG->data[index]), 100); // Cap at 100

            updated_cells.insert(index); // Add updated cell to the set
        }
    }

    // Decrease probability for other cells that have been updated by Lidar scan
    for (int index : updated_cells) {
        OG->data[index] -= decrease_probability_; // Decrease probability
        OG->data[index] = std::max(static_cast<int>(OG->data[index]), 0); // Cap at 0

    }

    // Publish the updated occupancy map
    updated_occupancy_map_publisher_->publish(*OG);
    RCLCPP_INFO(this->get_logger(), "Published Updated Occupancy Map");
}

    std::vector<std::pair<double, double>> transformLidarPoints(const std::vector<std::pair<double, double>>& lidar_points,
                                                                const nav_msgs::msg::Odometry& odometry_msg) {
        RCLCPP_INFO(this->get_logger(), "Transforming LiDAR points to map frame");

        // Initialize transformed points vector
        std::vector<std::pair<double, double>> transformed_points;

        // Extract robot's position from odometry message
        double robot_x = odometry_msg.pose.pose.position.x;
        double robot_y = odometry_msg.pose.pose.position.y;

        // Extract robot's orientation from odometry message
        double roll, pitch, yaw;
        tf2::Quaternion quaternion;
        tf2::fromMsg(odometry_msg.pose.pose.orientation, quaternion);
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        // Iterate through lidar points and transform each point to the map frame
        for (const auto& point : lidar_points) {
            // Transform lidar point from robot frame to map frame
            double map_x = robot_x + cos(yaw) * point.first - sin(yaw) * point.second;
            double map_y = robot_y + sin(yaw) * point.first + cos(yaw) * point.second;

            // Add transformed point to the vector
            transformed_points.push_back({map_x, map_y});
        }

        return transformed_points;
    }

    // ROS 2 subscriptions and publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_occupancy_map_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyMapUpdater>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

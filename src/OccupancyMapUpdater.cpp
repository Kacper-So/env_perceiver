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
        // Subscribe to occupancy map, LiDAR, and odometry topics
        occupancy_map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&OccupancyMapUpdater::occupancyMapCallback, this, std::placeholders::_1));
        rclcpp::QoS qos(rclcpp::KeepLast(5)); // Example QoS settings
        qos.best_effort(); // Set the reliability to best effort
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", qos, std::bind(&OccupancyMapUpdater::lidarCallback, this, std::placeholders::_1));

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&OccupancyMapUpdater::odometryCallback, this, std::placeholders::_1));

        // Create publisher for updated occupancy map
        updated_occupancy_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/updated_map", 10);

        // Initialize parameters
        epsilon_ = 0.2;  // Adjust epsilon according to your LiDAR sensor's resolution
        min_points_ = 5;  // Adjust min_points as needed
    }

private:
    nav_msgs::msg::OccupancyGrid OG;
    nav_msgs::msg::OccupancyGrid updated_OG;
    nav_msgs::msg::Odometry curr_odometry;
    double epsilon_;
    int min_points_;
    float map_origin_x_;
    float map_origin_y_;
    float map_resolution_;
    float map_width_;

    void occupancyMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_msg) {
        RCLCPP_INFO(this->get_logger(), "Received occupancy map");
        
        OG = *occupancy_map_msg;
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
        
        // Process LiDAR data
        // Convert sensor_msgs::msg::LaserScan to vector of points
        std::vector<std::pair<double, double>> points;
        for (float angle = lidar_msg->angle_min; angle < lidar_msg->angle_max; angle += lidar_msg->angle_increment) {
            double x = lidar_msg->range_min * cos(angle);
            double y = lidar_msg->range_min * sin(angle);
            points.push_back({x, y});
        }
        RCLCPP_INFO(this->get_logger(), "Number of LiDAR Points: %zu", points.size());

        // Transform LiDAR points to map frame
        std::vector<std::pair<double, double>> transformed_points = transformLidarPoints(points, curr_odometry);
        RCLCPP_INFO(this->get_logger(), "Number of Transformed Points: %zu", transformed_points.size());

        // Perform clustering using DBSCAN
        std::vector<std::vector<std::pair<double, double>>> clusters = fbscan(transformed_points);
        RCLCPP_INFO(this->get_logger(), "Number of Clusters: %zu", clusters.size());

        // Update occupancy grid with clustered LiDAR data
        updated_OG = OG;
        for (const auto& cluster : clusters) {
            for (const auto& point : cluster) {
                // Convert point to grid cell coordinates
                int grid_x = static_cast<int>((point.first - map_origin_x_) / map_resolution_);
                int grid_y = static_cast<int>((point.second - map_origin_y_) / map_resolution_);

                updated_OG.data[grid_y * map_width_ + grid_x] = 100; // Occupied cell
            }
        }

        // Publish the updated occupancy map
        updated_occupancy_map_publisher_->publish(updated_OG);
        RCLCPP_INFO(this->get_logger(), "Published Updated Occupancy Map");
    }
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        
        curr_odometry = *odometry_msg;
    }

    // Implement DBSCAN clustering algorithm
    std::vector<std::vector<std::pair<double, double>>> fbscan(const std::vector<std::pair<double, double>>& points) {
        RCLCPP_INFO(this->get_logger(), "Performing clustering using DBSCAN");
        
        std::vector<std::vector<std::pair<double, double>>> clusters;
        std::vector<bool> visited(points.size(), false);
        std::vector<int> cluster(points.size(), -1);
        int current_cluster = 0;

        for (size_t i = 0; i < points.size(); ++i) {
            if (!visited[i]) {
                visited[i] = true;
                std::vector<size_t> neighbor_points = regionQuery(points, i);
                if (neighbor_points.size() < min_points_) {
                    // Noise point
                    continue;
                }
                // Expand cluster
                clusters.push_back({});
                cluster[i] = current_cluster;
                clusters[current_cluster].push_back(points[i]);
                for (size_t j = 0; j < neighbor_points.size(); ++j) {
                    size_t neighbor_index = neighbor_points[j];
                    if (!visited[neighbor_index]) {
                        visited[neighbor_index] = true;
                        std::vector<size_t> neighbor_neighbor_points = regionQuery(points, neighbor_index);
                        if (neighbor_neighbor_points.size() >= min_points_) {
                            neighbor_points.insert(neighbor_points.end(), neighbor_neighbor_points.begin(), neighbor_neighbor_points.end());
                        }
                    }
                    if (cluster[neighbor_index] == -1) {
                        cluster[neighbor_index] = current_cluster;
                        clusters[current_cluster].push_back(points[neighbor_index]);
                    }
                }
                current_cluster++;
            }
        }

        return clusters;
        RCLCPP_INFO(this->get_logger(), "Done clustering using DBSCAN");
    }

    std::vector<size_t> regionQuery(const std::vector<std::pair<double, double>>& points, size_t index) {
        std::vector<size_t> neighbors;
        for (size_t i = 0; i < points.size(); ++i) {
            if (i != index) {
                double distance = std::sqrt(std::pow(points[i].first - points[index].first, 2) +
                                            std::pow(points[i].second - points[index].second, 2));
                if (distance <= epsilon_) {
                    neighbors.push_back(i);
                }
            }
        }
        return neighbors;
    }

    std::vector<std::pair<double, double>> transformLidarPoints(const std::vector<std::pair<double, double>>& lidar_points,
                                                                const nav_msgs::msg::Odometry odometry_msg) {
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

    // Define other necessary variables and functions here
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyMapUpdater>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

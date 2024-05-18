#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp" // Include for PointCloud2 message
#include "geometry_msgs/msg/point32.hpp"    // Include for Point32 message
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

using namespace std::chrono_literals;

enum class ClusterType {
    OBSTACLE,
    MAP_BORDER
};

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

        lidar_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", 10);

        transformed_lidar_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/transformed_lidar_points", 10);

        car_position_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/car_position", 10);
            
        trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
            "/planning/racing_planner/trajectory", 10, std::bind(&OccupancyMapUpdater::trajectoryCallback, this, std::placeholders::_1));

        obstacle_detected_publisher_ = this->create_publisher<std_msgs::msg::String>("/obstacle_detected", 10);
        obstacle_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_points", 10);
        obstacle_detection_iter_ = 0;

        //parameters
        obstacle_detection_threshold_ = 150; // Adjust the threshold as needed
        epsilon_ = 0.2;  // Adjust epsilon according to your LiDAR sensor's resolution
        min_points_ = 5;  // Adjust min_points as needed
        min_fov = -3.14 / 6;
        max_fov = 3.14 / 6;
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr OG;
    nav_msgs::msg::OccupancyGrid updated_OG;
    nav_msgs::msg::Odometry::SharedPtr curr_odometry;
    autoware_auto_planning_msgs::msg::Trajectory::SharedPtr curr_trajectory;
    bool trajectory_loaded = false;
    double epsilon_;
    int obstacle_detection_threshold_;
    int obstacle_detection_iter_;
    int min_points_;
    float map_origin_x_;
    float map_origin_y_;
    float map_resolution_;
    float map_width_;
    double min_fov;
    double max_fov;

    void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr trajectory_msg) {
        if(!trajectory_loaded){
            curr_trajectory = trajectory_msg;
            trajectory_loaded = true;
            //RCLCPP_INFO(this->get_logger(), curr_trajectory);
        }
    }

    void occupancyMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_msg) {
        if (!OG) {
            OG = occupancy_map_msg;
            map_origin_x_ = occupancy_map_msg->info.origin.position.x;
            map_origin_y_ = occupancy_map_msg->info.origin.position.y;
            map_resolution_ = occupancy_map_msg->info.resolution;
            map_width_ = occupancy_map_msg->info.width;
        }
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg) {
        if (!OG) {
            return;
        }

        std::vector<std::pair<double, double>> points;
        for (size_t i = 0; i < lidar_msg->ranges.size(); ++i) {
            double angle = lidar_msg->angle_min + i * lidar_msg->angle_increment;
            if (angle < min_fov || angle > max_fov) {
                continue;
            }
            double range = lidar_msg->ranges[i];
            if (range < lidar_msg->range_min || range > lidar_msg->range_max)
                continue;
            double x = range * cos(angle);
            double y = range * sin(angle);
            points.push_back({x, y});
        }

        publishPointCloud(lidar_points_publisher_, points);

        std::vector<std::pair<double, double>> transformed_points = transformLidarPoints(points, *curr_odometry);

        publishPointCloud(transformed_lidar_points_publisher_, transformed_points);

        std::vector<std::pair<double, double>> car_position = {{curr_odometry->pose.pose.position.x, curr_odometry->pose.pose.position.y}};
        publishPointCloud(car_position_publisher_, car_position);

        std::vector<std::vector<std::pair<double, double>>> clusters = fbscan(transformed_points);
        RCLCPP_INFO(this->get_logger(), "Number of Clusters: %zu", clusters.size());

        updated_OG = *OG;
        for (const auto& cluster : clusters) {
            for (const auto& point : cluster) {
                int grid_x = static_cast<int>((point.first - map_origin_x_) / map_resolution_);
                int grid_y = static_cast<int>((point.second - map_origin_y_) / map_resolution_);

                if (grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < updated_OG.info.height) {
                    updated_OG.data[grid_y * map_width_ + grid_x] = 100; // Occupied cell
                }
            }
        }

        for (const auto& cluster : clusters) {
            bool is_obstacle = isClusterObstacle(cluster);
            if (is_obstacle) {
                publishPointCloud(obstacle_points_publisher_, cluster); // Publish obstacle clusters as PointCloud
            }
            if (is_obstacle) {
                obstacle_detection_iter_++;
                if (obstacle_detection_iter_ > obstacle_detection_threshold_) {
                    std_msgs::msg::String msg;
                    msg.data = "Obstacle detected!";
                    obstacle_detected_publisher_->publish(msg);
                    obstacle_detection_iter_ = 0;
                }
            }
        }

        updated_occupancy_map_publisher_->publish(updated_OG);
    }

    bool isClusterObstacle(const std::vector<std::pair<double, double>>& cluster) {
        double centroid_x = 0.0, centroid_y = 0.0;
        for (const auto& point : cluster) {
            centroid_x += point.first;
            centroid_y += point.second;
        }
        centroid_x /= cluster.size();
        centroid_y /= cluster.size();

        double angle_cluster_car = std::atan2(centroid_y - curr_odometry->pose.pose.position.y,
                                              centroid_x - curr_odometry->pose.pose.position.x);
        double car_orientation = tf2::getYaw(curr_odometry->pose.pose.orientation);
        double angle_diff = std::abs(car_orientation - angle_cluster_car);
        if(angle_diff < (M_PI / 12)){
            for (const auto& point : cluster) {
                for (const auto& traj_point : curr_trajectory->points) {
                    double dist = std::sqrt(std::pow(point.first - traj_point.pose.position.x, 2) +
                                            std::pow(point.second - traj_point.pose.position.y, 2));
                    if (dist < 0.05 ) { // Check if the cluster point is close to any trajectory point//epsilon_
                        return true;
                    }
                }
            }
            return false;
        }

        return false;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
        curr_odometry = odometry_msg;
    }

    std::vector<std::vector<std::pair<double, double>>> fbscan(const std::vector<std::pair<double, double>>& points) {
        std::vector<std::vector<std::pair<double, double>>> clusters;
        std::vector<bool> visited(points.size(), false);
        std::vector<int> cluster(points.size(), -1);
        int current_cluster = 0;

        for (size_t i = 0; i < points.size(); ++i) {
            if (!visited[i]) {
                visited[i] = true;
                std::vector<size_t> neighbor_points = regionQuery(points, i);
                if (neighbor_points.size() < min_points_) {
                    continue;
                }
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
                                                                const nav_msgs::msg::Odometry& odometry_msg) {
        std::vector<std::pair<double, double>> transformed_points;

        double robot_x = odometry_msg.pose.pose.position.x;
        double robot_y = odometry_msg.pose.pose.position.y;

        double roll, pitch, yaw;
        tf2::Quaternion quaternion;
        tf2::fromMsg(odometry_msg.pose.pose.orientation, quaternion);
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        for (const auto& point : lidar_points) {
            double map_x = robot_x + cos(yaw) * point.first - sin(yaw) * point.second;
            double map_y = robot_y + sin(yaw) * point.first + cos(yaw) * point.second;
            transformed_points.push_back({map_x, map_y});
        }

        return transformed_points;
    }

    void publishPointCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
                        const std::vector<std::pair<double, double>>& points) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& point : points) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.first;
            pcl_point.y = point.second;
            pcl_point.z = 0.0; // Adjust z value if needed
            cloud->push_back(pcl_point);
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);

        output.header.frame_id = "map"; // Adjust frame ID if needed
        output.header.stamp = this->now();

        publisher->publish(output);
    }

    // ROS 2 subscriptions and publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;

    // ROS 2 publishers for point clouds
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_occupancy_map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_lidar_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr car_position_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_detected_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_points_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyMapUpdater>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
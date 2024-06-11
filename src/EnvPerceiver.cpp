#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include <vector>
#include <tuple>
#include <queue>
#include <unordered_map>

using namespace std::chrono_literals;

struct Cell {
    int x;
    int y;
    bool operator<(const Cell& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
};

enum class ClusterType {
    OBSTACLE,
    MAP_BORDER
};

class EnvPerceiver : public rclcpp::Node {
public:
    EnvPerceiver() : Node("env_perceiver") {
        this->declare_parameter<double>("epsilon", 0.05);
        this->declare_parameter<int>("obstacle_detection_threshold", 150);
        this->declare_parameter<int>("min_points", 5);
        this->declare_parameter<double>("min_fov", -M_PI / 6);
        this->declare_parameter<double>("max_fov", M_PI / 6);

        epsilon_ = this->get_parameter("epsilon").as_double();
        obstacle_detection_threshold_ = this->get_parameter("obstacle_detection_threshold").as_int();
        min_points_ = this->get_parameter("min_points").as_int();
        min_fov = this->get_parameter("min_fov").as_double();
        max_fov = this->get_parameter("max_fov").as_double();

        occupancy_map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&EnvPerceiver::occupancyMapCallback, this, std::placeholders::_1));
        
        rclcpp::QoS qos(rclcpp::KeepLast(5)); // Example QoS settings
        qos.best_effort(); // Set the reliability to best effort
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", qos, std::bind(&EnvPerceiver::lidarCallback, this, std::placeholders::_1));

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&EnvPerceiver::odometryCallback, this, std::placeholders::_1));

        updated_occupancy_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/local_occupancy_grid", 10);

        trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
            "/traj", 10, std::bind(&EnvPerceiver::trajectoryCallback, this, std::placeholders::_1));

        start_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/start_point", 10);
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/goal_point", 10);
        obstacle_detected_publisher_ = this->create_publisher<std_msgs::msg::String>("/obstacle_alarm", 10);
        
        start_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/start_pointcloud", 10);
        goal_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/goal_pointcloud", 10);
        border_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/border_pointcloud", 10);

        obstacle_detection_iter_ = 0;
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
    std::vector<geometry_msgs::msg::Point> border;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr start_pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr goal_pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr border_pointcloud_publisher_;

    void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr trajectory_msg) {
        if(!trajectory_loaded){
            curr_trajectory = trajectory_msg;
            trajectory_loaded = true;
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

    void rotatePoint(int &x, int &y, double angle, int cx, int cy) {
        double rad = angle;
        double cosAngle = std::cos(rad);
        double sinAngle = std::sin(rad);
        int nx = std::round(cosAngle * (x - cx) - sinAngle * (y - cy) + cx);
        int ny = std::round(sinAngle * (x - cx) + cosAngle * (y - cy) + cy);
        x = nx;
        y = ny;
    }

    void limitToSquare(nav_msgs::msg::OccupancyGrid &grid, int square_size, std::pair<int, int> center_point) {
        int width = grid.info.width;
        int height = grid.info.height;

        int half_size = square_size / 2;
        int start_x = std::max(10, center_point.first);
        int start_y = std::max(0, center_point.second - half_size);

        // Ensure the square doesn't go out of bounds
        if (start_x + square_size > width) {
            start_x = width - square_size;
        }
        if (start_y + square_size > height) {
            start_y = height - square_size;
        }

        std::vector<int8_t> new_data(square_size * square_size, 0);
        int center_x = center_point.first;
        int center_y = center_point.second;

        for (int y = 0; y < square_size; ++y) {
            for (int x = 0; x < square_size; ++x) {
                int src_x = start_x + x;
                int src_y = start_y + y;
                rotatePoint(src_x, src_y, tf2::getYaw(curr_odometry->pose.pose.orientation), center_x, center_y);
                
                if (src_x >= 0 && src_x < width && src_y >= 0 && src_y < height) {
                    new_data[y * square_size + x] = grid.data[src_y * width + src_x];
                } else {
                    new_data[y * square_size + x] = -1; // Unknown area, can set to -1 or any default value
                }
            }
        }

        grid.data = new_data;
        grid.info.width = square_size;
        grid.info.height = square_size;
    }
    
    void fillOgGradient(nav_msgs::msg::OccupancyGrid &grid) {
        int width = grid.info.width;
        int height = grid.info.height;
        auto &data = grid.data;
        // Initialize distance grid with a large value
        std::vector<std::vector<int>> distance_grid(height, std::vector<int>(width, std::numeric_limits<int>::max()));
        // Create a queue for BFS
        std::queue<Cell> q;
        // Add all cells with value 100 to the queue and set their distance to 0
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (data[y * width + x] == 100) {
                    q.push({x, y});
                    distance_grid[y][x] = 0;
                }
            }
        }
        // Directions for moving in the grid (right, down, left, up)
        std::vector<Cell> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        // Perform BFS
        while (!q.empty()) {
            Cell current = q.front();
            q.pop();
            int current_distance = distance_grid[current.y][current.x];

            for (const Cell &dir : directions) {
                int new_x = current.x + dir.x;
                int new_y = current.y + dir.y;

                // Check if the new cell is within the grid bounds
                if (new_x >= 0 && new_x < width && new_y >= 0 && new_y < height) {
                    // Update the distance if a shorter path is found
                    if (distance_grid[new_y][new_x] > current_distance + 1) {
                        distance_grid[new_y][new_x] = current_distance + 1;
                        q.push({new_x, new_y});
                    }
                }
            }
        }
        // Update the original grid with the calculated distances
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (data[y * width + x] == 0) {
                    data[y * width + x] = 100 - distance_grid[y][x];
                    if (data[y * width + x] < 0) {data[y * width + x] = 0;}
                }
            }
        }
        // Fill top and bottom borders
        for (int x = 0; x < width; ++x){
            data[x] = 100; // Top border
            data[(height - 1) * width + x] = 100; // Bottom border
        }
        // Fill left and right borders
        for (int y = 0; y < height; ++y){
            data[y * width] = 100; // Left border
            data[y * width + (width - 1)] = 100; // Right border
        }
    }
    
    void findBorderCells(nav_msgs::msg::OccupancyGrid &grid){
        border.clear();
        int width = grid.info.width;
        int height = grid.info.height;
        for (int y = 0; y < height; ++y){
            for (int x = 0; x < width; ++x){
                if (grid.data[y * width + x] == 100){
                    geometry_msgs::msg::Point point;
                    point.x = static_cast<double>(x) * grid.info.resolution + grid.info.origin.position.x;
                    point.y = static_cast<double>(y) * grid.info.resolution + grid.info.origin.position.y;
                    point.z = 0.0;
                    border.push_back(point);
                }
            }
        }
        publishPointCloud(border_pointcloud_publisher_, border);
    }
    
    Cell findEndPoint(const nav_msgs::msg::OccupancyGrid& OG, const Cell& start) {
        int width = OG.info.width;
        int height = OG.info.height;

        int x_mid = (width / 2);

        int min_value = std::numeric_limits<int>::max();
        Cell end_point;

        for (int y = 0; y < height; ++y) {
            for (int x = x_mid; x < width - 10; ++x) {
                int index = y * width + x;
                if (OG.data[index] < min_value && OG.data[index] != -1) {
                    min_value = OG.data[index];
                    end_point = {x, y};
                }
            }
        }
        return end_point;
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
            points.push_back({x + 0.35, y});
        }

        std::vector<std::pair<double, double>> transformed_points = transformLidarPoints(points, *curr_odometry);

        std::vector<std::pair<double, double>> car_position = {{curr_odometry->pose.pose.position.x, curr_odometry->pose.pose.position.y}};

        std::vector<std::vector<std::pair<double, double>>> clusters = fbscan(transformed_points);

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
                obstacle_detection_iter_++;
                if (obstacle_detection_iter_ > obstacle_detection_threshold_) {
                    std_msgs::msg::String msg;
                    msg.data = "Obstacle detected!";
                    obstacle_detected_publisher_->publish(msg);
                    obstacle_detection_iter_ = 0;
                }
            }
        }

        double map_origin_x = updated_OG.info.origin.position.x;
        double map_origin_y = updated_OG.info.origin.position.y;
        double resolution = updated_OG.info.resolution;
        int width = updated_OG.info.width;
        int height = updated_OG.info.height;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                if (updated_OG.data[index] != 0 && updated_OG.data[index] != -1) {
                    updated_OG.data[index] = 100;
                }
            }
        }
        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                int index = y * width + x;
                if (updated_OG.data[index] == 100) {
                    bool border_only = true;
                    // Check the 4 neighboring cells in the x/y plane
                    int neighbors[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
                    for (auto& n : neighbors) {
                        int nx = x + n[0];
                        int ny = y + n[1];
                        int neighbor_index = ny * width + nx;
                        if (updated_OG.data[neighbor_index] != 100 && updated_OG.data[neighbor_index] != -1) {
                            border_only = false;
                            break;
                        }
                    }
                    if (border_only) {
                        updated_OG.data[index] = -1;
                    }
                }
            }
        }

        int grid_x = static_cast<int>((curr_odometry->pose.pose.position.x - map_origin_x) / resolution);
        int grid_y = static_cast<int>((curr_odometry->pose.pose.position.y - map_origin_y) / resolution);
        std::pair<int, int> center_point = {grid_x, grid_y};
        int square_size = 100;
        limitToSquare(updated_OG, square_size, center_point);
        map_origin_x = updated_OG.info.origin.position.x;
        map_origin_y = updated_OG.info.origin.position.y;
        resolution = updated_OG.info.resolution;
        width = updated_OG.info.width;
        fillOgGradient(updated_OG);
        findBorderCells(updated_OG);
        Cell start{10, square_size / 2};
        Cell goal = findEndPoint(updated_OG, start);
        geometry_msgs::msg::Pose start_gm;
        geometry_msgs::msg::Pose goal_gm;
        start_gm.position.x = start.x * resolution + map_origin_x;
        start_gm.position.y = start.y * resolution + map_origin_y;
        goal_gm.position.x = goal.x * resolution + map_origin_x;
        goal_gm.position.y = goal.y * resolution + map_origin_y;
        start_gm.orientation = curr_odometry->pose.pose.orientation;
        goal_gm.orientation = curr_odometry->pose.pose.orientation;
        start_publisher_->publish(start_gm);
        goal_publisher_->publish(goal_gm);

        // Publish start and goal points as point clouds
        publishSinglePointPointCloud(start_pointcloud_publisher_, start_gm.position);
        publishSinglePointPointCloud(goal_pointcloud_publisher_, goal_gm.position);
        
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
        if(angle_diff < (M_PI / 16)){
            for (const auto& point : cluster) {
                for (const auto& traj_point : curr_trajectory->points) {
                    double dist = std::sqrt(std::pow(point.first - traj_point.pose.position.x, 2) +
                                            std::pow(point.second - traj_point.pose.position.y, 2));
                    if (dist < epsilon_ ) { // Check if the cluster point is close to any trajectory point//epsilon_
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
                        const std::vector<geometry_msgs::msg::Point>& points) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& point : points) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z; // Adjust z value if needed
            cloud->push_back(pcl_point);
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);

        output.header.frame_id = "map"; // Adjust frame ID if needed
        output.header.stamp = this->now();

        publisher->publish(output);
    }

    void publishSinglePointPointCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
                                      const geometry_msgs::msg::Point& point) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z; // Adjust z value if needed
        cloud->push_back(pcl_point);

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

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_detected_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr start_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EnvPerceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
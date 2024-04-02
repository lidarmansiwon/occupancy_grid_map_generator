#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <iostream>

class OccupancyGridMapGenerator : public rclcpp::Node
{
public:
    OccupancyGridMapGenerator() : Node("occupancy_grid_map_generator")
    {   

        // Declare parameters with default values
        declare_parameters();

        // Get parameter values
        get_parameters();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 센서 데이터에 대한 신뢰할 수 있는 QoS 설정

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/filtered_points", qos,
            std::bind(&OccupancyGridMapGenerator::pointcloud_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid_topic", 10);
    }

private:

    float min_x_; // Minimum x coordinate of the point cloud
    float max_x_; // Maximum x coordinate of the point cloud
    float min_y_; // Minimum y coordinate of the point cloud
    float max_y_; // Maximum y coordinate of the point cloud
    float grid_size_; // Grid size in meters
    float origin_orientation_x;
    float origin_orientation_y;
    float origin_orientation_z;
    float origin_orientation_w;
    float safety_radius_;   // 전역 변수로 safety_radius 설정

    float boat_position_x_; // 보트의 x 좌표
    float boat_position_y_; // 보트의 y 좌표
    float boat_radius_;     // 보트 안전 반경
    float boat_width_;     // 보트 안전 반경
    float boat_height_;     // 보트 안전 반경

    void declare_parameters()
    {   
        declare_parameter<float>("min_x_", 0.0);
        declare_parameter<float>("max_x_", 10.0);
        declare_parameter<float>("min_y_", 0.0);
        declare_parameter<float>("max_y_", 10.0);
        declare_parameter<float>("grid_size_", 0.1);
        declare_parameter<float>("origin_orientation_x", 0.0);
        declare_parameter<float>("origin_orientation_y", 0.0);
        declare_parameter<float>("origin_orientation_z", 0.0);
        declare_parameter<float>("origin_orientation_w", 1.0);
        declare_parameter<float>("safety_radius_", 0.7);
        declare_parameter<float>("boat_position_x_", 0.0);
        declare_parameter<float>("boat_position_y_", 0.0);
        declare_parameter<float>("boat_radius_", 0.8);
        declare_parameter<float>("boat_width_", 0.8);
        declare_parameter<float>("boat_height_", 2.0);

    }

    void get_parameters()
    {   
        get_parameter("min_x_", min_x_);
        get_parameter("max_x_", max_x_);
        get_parameter("min_y_", min_y_);
        get_parameter("max_y_", max_y_);
        get_parameter("grid_size_", grid_size_);

        get_parameter("origin_orientation_x", origin_orientation_x);
        get_parameter("origin_orientation_y", origin_orientation_y);
        get_parameter("origin_orientation_z", origin_orientation_z);
        get_parameter("origin_orientation_w", origin_orientation_w);
        get_parameter("safety_radius_", safety_radius_);
        get_parameter("boat_position_x_", boat_position_x_);
        get_parameter("boat_position_y_", boat_position_y_);
        get_parameter("boat_radius_", boat_radius_);
        get_parameter("boat_width_", boat_width_);
        get_parameter("boat_height_", boat_height_);
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PCLPointCloud2 pcl_cloud;
        pcl_conversions::toPCL(*msg, pcl_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_cloud, *pcl_cloud_xyz);

        // Voxel grid downsampling (optional, for efficiency)
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(pcl_cloud_xyz);
        vg.setLeafSize(0.5, 0.5, 0.5); // Adjust the leaf size according to your preference
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*pcl_cloud_downsampled);
    
        // Convert PointCloud to OccupancyGrid
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        
        // Assuming grid parameters
        float min_x = min_x_; // Minimum x coordinate of the point cloud
        float max_x = max_x_; // Maximum x coordinate of the point cloud
        float min_y = min_y_; // Minimum y coordinate of the point cloud
        float max_y = max_y_; // Maximum y coordinate of the point cloud
        float grid_size = grid_size_; // Grid size in meters
        // int grid_width = static_cast<int>((max_x - min_x) / grid_size) + 1; // Number of grid cells along x-axis
        // int grid_height = static_cast<int>((max_y - min_y) / grid_size) + 1; // Number of grid cells along y-axis
        int grid_width = static_cast<int>((max_x - min_x) / grid_size) + 1;
        int grid_height = static_cast<int>((max_y - min_y) / grid_size) + 1;

        // 전역 변수로 safety_radius 설정
        float safety_radius = safety_radius_; // 단위: 미터

        // Initialize occupancy grid map
        occupancy_grid.header.stamp = this->now(); // Current time
        occupancy_grid.header.frame_id = "map"; // Frame ID
        occupancy_grid.info.width = grid_width; // Number of cells in the grid along the x-axis
        occupancy_grid.info.height = grid_height; // Number of cells in the grid along the y-axis
        occupancy_grid.info.resolution = grid_size; // Size of each grid cell
        occupancy_grid.info.origin.position.x = min_x; // Origin of the grid map along the x-axis
        occupancy_grid.info.origin.position.y = min_y; // Origin of the grid map along the y-axis
        occupancy_grid.info.origin.position.z = 0.0; // Origin of the grid map along the z-axis (assuming 2D map)
        occupancy_grid.info.origin.orientation.x = origin_orientation_x;
        occupancy_grid.info.origin.orientation.y = origin_orientation_y; // sin(45 degrees / 2) = 0.707
        occupancy_grid.info.origin.orientation.z = origin_orientation_z;
        occupancy_grid.info.origin.orientation.w = origin_orientation_w; // cos(45 degrees / 2) = 0.707


        float boat_position_x = boat_position_x_;
        float boat_position_y = boat_position_y_;
        float boat_radius     = boat_radius_;
        float boat_width      = boat_width_;                    // 보트의 가로 크기 (단위: 미터)
        float boat_height     = boat_height_;                   // 보트의 세로 크기 (단위: 미터)


        // Assuming occupancy grid values based on point cloud data
        for (int i = 0; i < grid_width; ++i) {
            for (int j = 0; j < grid_height; ++j) {
                // Calculate the center of the current grid cell
                float cell_center_x = min_x + (i + 0.5) * grid_size;
                float cell_center_y = min_y + (j + 0.5) * grid_size;

                // Calculate the distance between the cell center and the robot position
                float distance_to_boat_x = abs(boat_position_x - cell_center_x);  // fabs 부호를 무시하고 항상 양수 값을 반환
                float distance_to_boat_y = abs(boat_position_y - cell_center_y);

                // Check if the distance to the robot is within the specified threshold
                if (distance_to_boat_x <= boat_height / 2 && distance_to_boat_y <= boat_width / 2) {
                    occupancy_grid.data.push_back(100); // Occupied cell
                } else {
                    // Check if any point in the vicinity of the grid cell is occupied
                    bool cell_occupied = false;
                    for (const auto& point : *pcl_cloud_downsampled) {
                        float point_x = point.x;
                        float point_y = point.y;

                        // Check if the point falls within the vicinity of the cell, including the safety radius
                        if (point_x >= cell_center_x - safety_radius && point_x < cell_center_x + safety_radius &&
                            point_y >= cell_center_y - safety_radius && point_y < cell_center_y + safety_radius) {
                            cell_occupied = true;
                            break;
                        }
                    }
                    // Set occupancy value based on whether the cell is occupied or not
                    if (cell_occupied) {
                        occupancy_grid.data.push_back(100); // Occupied cell

                    } else {
                        occupancy_grid.data.push_back(0); // Free cell
                        // RCLCPP_INFO(this->get_logger(), "occupancy_grid: %f", occupancy_grid.data, "");
                    }
                }
            }
        }

        // for (const auto& value : occupancy_grid.data) {
        //     RCLCPP_INFO(this->get_logger(), "occupancy_grid: %d", value);
        //     }
        publisher_->publish(occupancy_grid);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyGridMapGenerator>();

    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

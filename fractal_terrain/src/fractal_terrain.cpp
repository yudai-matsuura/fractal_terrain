#include "fractal_terrain/fractal_terrain.hpp"

FractalTerrain::FractalTerrain(const rclcpp::NodeOptions & options)
: rclcpp::Node("fractal_terrain", options)
{
    // Parameter
    this->declare_parameter<double>("min_x", 2.4);
    this->declare_parameter<double>("max_x", 3.1);
    this->declare_parameter<double>("min_y", 0.7);
    this->declare_parameter<double>("max_y", 1.4);
    this->declare_parameter<double>("resolution", 0.005);
    this->declare_parameter<std::string>("frame_id", "tcp_base");
    this->declare_parameter<int>("base_grid_size", 128);
    this->declare_parameter<double>("target_std", 0.02);  // target Stdev
    this->declare_parameter<double>("fractal_omega", -3.0);
    this->declare_parameter<std::string>("csv_filename", "fractal_terrain.csv");

    // Get parameter
    int base_grid_size = this->get_parameter("base_grid_size").as_int();
    double target_std = this->get_parameter("target_std").as_double();
    double omega = this->get_parameter("fractal_omega").as_double();
    csv_filename_ = this->get_parameter("csv_filename").as_string();

    // Publisher
    pub_terrain_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("virtual_terrain", 10);

    RCLCPP_INFO(this->get_logger(), "Generating Fractal Terrain (Omega: %.1f, STD: %.4f)...", omega, target_std);
    terrain_base_ = generateFractalTerrain(base_grid_size, static_cast<float>(omega), static_cast<float>(target_std));

    // Save terrain sa CSV
    saveTerrainAsCSV(
        terrain_base_, this->get_parameter("min_x").as_double(),
        this->get_parameter("min_y").as_double(), this->get_parameter("max_x").as_double(),
        this->get_parameter("max_y").as_double(), csv_filename_);
    RCLCPP_INFO(this->get_logger(), "Terrain saved to %s", csv_filename_.c_str());

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&FractalTerrainPublisher::timer_callback, this));

}
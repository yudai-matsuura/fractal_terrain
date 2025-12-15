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
    virtual_terrain_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("virtual_terrain", 10);

    RCLCPP_INFO(this->get_logger(), "Generating Fractal Terrain (Omega: %.1f, STD: %.4f)...", omega, target_std);
    terrain_base_ = generateFractalTerrain(base_grid_size, static_cast<float>(omega), static_cast<float>(target_std));

    // Save terrain sa CSV
    saveTerrainAsCSV(
        terrain_base_, this->get_parameter("min_x").as_double(),
        this->get_parameter("min_y").as_double(), this->get_parameter("max_x").as_double(),
        this->get_parameter("max_y").as_double(), csv_filename_);
    RCLCPP_INFO(this->get_logger(), "Terrain saved to %s", csv_filename_.c_str());

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&FractalTerrainPublisher::timerCallback, this));
}

FractalTerrain::~FractalTerrain()
{
  std::cout << "FractalTerrain class is destructed." << std::endl;
}


std::vector<double> FractalTerrain::linspace(double start, double end, int num)
{
  std::vector<double> vec;
  if (num <= 0) {
    return vec;
  }

  if (num == 1) {
    vec.push_back(start);
    return vec;
  }

  double step = (end - start) / (num - 1);
  for (int i = 0; i < num; ++i) {
    vec.push_back(start + (static_cast<double>(i) * step));
  }
  return vec;
}

cv::Mat FractalTerrain::generateFractalTerrain(int size, float omega, float target_std)
{
  // Add fractal terrain generation code
}

void FractalTerrain::saveTerrainAsCSV(const cv::Mat & terrain, double min_x, double min_y, double max_x, double max_y,
  const std::string & filename)
{
  int num_x = terrain.cols;
  int num_y = terrain.rows;
  double dx = (max_x - min_x) / (num_x - 1);
  double dy = (max_y - min_y) / (num_y - 1);

  std::ofstream ofs(filename);
  ofs << "x,y,z\n";

  for (int j = 0; j < num_y; j++) {
    for (int i = 0; i < num_x; i++) {
      float z = terrain.at<float>(j, i);
      double x = min_x + i * dx;
      double y = min_y + j * dy;
      ofs << x << "," << y << "," << z << "\n";
    }
  }
  ofs.close();
}

void FractalTerrain::timerCallback()
{
  auto msg = createPointCloudMsg(terrain_base_);
  virtual_terrain_pub_->publish(msg);
}

sensor_msgs::msg::PointCloud2 FractalTerrain::createPointCloudMsg(const cv:Mat & base_terrain)
{
  // Add cloud generation code
}
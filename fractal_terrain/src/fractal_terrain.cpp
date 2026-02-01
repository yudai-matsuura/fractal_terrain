#include "fractal_terrain/fractal_terrain.hpp"

FractalTerrain::FractalTerrain(const rclcpp::NodeOptions & options)
: rclcpp::Node("fractal_terrain", options)
{
    // Default parameters
    this->declare_parameter<double>("min_x", 2.4);
    this->declare_parameter<double>("max_x", 3.1);
    this->declare_parameter<double>("min_y", 0.7);
    this->declare_parameter<double>("max_y", 1.4);
    this->declare_parameter<double>("resolution", 0.005);
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<int>("base_grid_size", 128);
    this->declare_parameter<double>("target_std", 0.02);
    this->declare_parameter<double>("fractal_omega", -4.0);
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

    // Save terrain as CSV
    saveTerrainAsCSV(
        terrain_base_, this->get_parameter("min_x").as_double(),
        this->get_parameter("min_y").as_double(), this->get_parameter("max_x").as_double(),
        this->get_parameter("max_y").as_double(), csv_filename_);
    RCLCPP_INFO(this->get_logger(), "Terrain saved to %s", csv_filename_.c_str());

    // STL
    double dx = 0.0005; // x resolution [m]
    double dy = 0.0005; // y resolution [m]
    std::string filename = "fractal_terrain.stl";
    saveMeshAsSTL(terrain_base_, dx, dy, filename);
    RCLCPP_INFO(this->get_logger(), "Mesh saved to %s", filename.c_str());

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&FractalTerrain::timerCallback, this));
}

FractalTerrain::~FractalTerrain()
{
  std::cout << "FractalTerrain class is destructed." << std::endl;
}


void FractalTerrain::timerCallback()
{
  auto msg = createPointCloudMsg(terrain_base_);
  virtual_terrain_pub_->publish(msg);
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
  // random noise
  cv::Mat noise(size, size, CV_32F);
  cv::randn(noise, 0.0f, 1.0f);

  // Create complex image
  cv::Mat dft_planes[] = {noise, cv::Mat::zeros(noise.size(), CV_32F)}; // A complex number with an imaginary part of 0
  cv::Mat complex_img;
  cv::merge(dft_planes, 2, complex_img);

  // Discrete Fourier Transform
  cv::dft(complex_img, complex_img);

  // Separate to real part and imaginary part
  cv::split(complex_img, dft_planes);
  int center_x = size / 2;
  int center_y = size / 2;

  // Amplitude redistribution for all frequency components based on fractal law
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      int i = (x <= center_x) ? x : x - size;
      int j = (y <= center_y) ? y : y - size;
      float freq = std::sqrt(static_cast<float>(i * i + j * j));
      float scale = (freq > 0.0f) ? std::pow(freq, omega / 2.0f) : 0.0f; // Amplitude is the square root of the power
      dft_planes[0].at<float>(y, x) *= scale;
      dft_planes[1].at<float>(y, x) *= scale;
    }
  }

    // Inverse Fourier Transform
    cv::merge(dft_planes, 2, complex_img);
    cv::dft(complex_img, complex_img, cv::DFT_INVERSE | cv::DFT_SCALE);
    cv::split(complex_img, dft_planes);
    cv::Mat fractal_terrain = dft_planes[0]; // Height map in real space

    // Standard deviation adjustment
    cv::Scalar mean, stddev;
    cv::meanStdDev(fractal_terrain, mean, stddev);
    float current_std = static_cast<float>(stddev[0]);
    float current_mean = static_cast<float>(mean[0]);
    if (current_std > 1e-6) {
      fractal_terrain = (fractal_terrain - current_mean) * (target_std / current_std);
    }

    // Check ln(E) value
    cv::Mat sq = fractal_terrain.mul(fractal_terrain);
    double energy_spatial = cv::sum(sq)[0];
    double log_energy_spatial = std::log(energy_spatial + 1e-12);
    std::cout << "energy_spatial = " << log_energy_spatial << std::endl;

    return fractal_terrain;
}

sensor_msgs::msg::PointCloud2 FractalTerrain::createPointCloudMsg(const cv::Mat & base_terrain)
{
  // Get parameters
  double min_x = this->get_parameter("min_x").as_double();
  double max_x = this->get_parameter("max_x").as_double();
  double min_y = this->get_parameter("min_y").as_double();
  double max_y = this->get_parameter("max_y").as_double();
  double resolution = this->get_parameter("resolution").as_double();
  std::string frame_id = this->get_parameter("frame_id").as_string();

  // Calculate the number of grid points
  int num_x = static_cast<int>((max_x - min_x) / resolution) + 1;
  int num_y = static_cast<int>((max_y - min_y) / resolution) + 1;

  if (num_x <= 0 || num_y <= 0) {
    return sensor_msgs::msg::PointCloud2();
  }

  std::vector<double> x_vec = linspace(min_x, max_x, num_x);
  std::vector<double> y_vec = linspace(min_y, max_y, num_y);

  //Resize fractal terrain
  cv::Mat resized_terrain;
  cv::resize(base_terrain, resized_terrain, cv::Size(num_x, num_y), 0, 0, cv::INTER_LINEAR);

  // Initialize point cloud message
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_x * num_y;
  msg.is_dense = true;
  msg.is_bigendian = false;

  // Field setting
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.point_step = 12;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  // Create point cloud
  uint8_t * ptr = msg.data.data();
  for (int i = 0; i < num_y; ++i) {
    for (int j = 0; j < num_x; ++j) {
      float z = resized_terrain.at<float>(i, j);
      PointXYZ p;
      p.x = static_cast<float>(x_vec[j]);
      p.y = static_cast<float>(y_vec[i]);
      p.z = z;
      std::memcpy(ptr, &p, sizeof(PointXYZ));
      ptr += sizeof(PointXYZ);
    }
  }

return msg;
}

void FractalTerrain::saveMeshAsSTL(const cv::Mat &terrain, double dx, double dy, const std::string &filename)
{
    std::ofstream ofs(filename);
    ofs << "solid terrain\n";

    int rows = terrain.rows;
    int cols = terrain.cols;

    auto writeTriangle = [&](cv::Point3d v0, cv::Point3d v1, cv::Point3d v2) {
        cv::Point3d normal = (v1 - v0).cross(v2 - v0);
        double norm = cv::norm(normal);
        if (norm > 1e-8) normal /= norm;
        ofs << "  facet normal " << normal.x << " " << normal.y << " " << normal.z << "\n";
        ofs << "    outer loop\n";
        ofs << "      vertex " << v0.x << " " << v0.y << " " << v0.z << "\n";
        ofs << "      vertex " << v1.x << " " << v1.y << " " << v1.z << "\n";
        ofs << "      vertex " << v2.x << " " << v2.y << " " << v2.z << "\n";
        ofs << "    endloop\n";
        ofs << "  endfacet\n";
    };

    for (int i = 0; i < rows-1; ++i) {
        for (int j = 0; j < cols-1; ++j) {
            cv::Point3d v00(j*dx, i*dy, terrain.at<float>(i,j));
            cv::Point3d v10((j+1)*dx, i*dy, terrain.at<float>(i+1,j));
            cv::Point3d v01(j*dx, (i+1)*dy, terrain.at<float>(i,j+1));
            cv::Point3d v11((j+1)*dx, (i+1)*dy, terrain.at<float>(i+1,j+1));

            // 2 triangles per cell
            writeTriangle(v00, v10, v01);
            writeTriangle(v10, v11, v01);
        }
    }

    ofs << "endsolid terrain\n";
    ofs.close();
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
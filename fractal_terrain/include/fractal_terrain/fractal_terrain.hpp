#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct PointXYZ
{
    float x;
    float y;
    float z;
};

class FractalTerrain : public rclcpp::Node
{
public:
    explicit FractalTerrain(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    virtual ~FractalTerrain();

private:
  /**
   * @brief This function generate evenly spaced numeric vectors.
   *
   * @param start
   * @param end
   * @param num
   */
    std::vector<double> linspace(double start, double end, int num);

	/**
   * @brief This function create virtual terrain using fractal law.
   *
   * @param size
   * @param omega
   * @param target_std
   */
	cv::Mat generateFractalTerrain(int size, float omega, float target_std);

	/**
   * @brief This function save fractal terrain data to csv file.
   *
   * @param terrain
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   * @param filename
   */
	void saveTerrainAsCSV(const cv::Mat & terrain, double min_x, double min_y, double max_x, double max_y,
	const std::string & filename);

	/**
   * @brief
   *
   */
	void timerCallback();

	/**
   * @brief This function create point cloud message from fractal terrain data.
   *
   * @param base_terrain
   */
  sensor_msgs::msg::PointCloud2 createPointCloudMsg(const cv::Mat & base_terrain);

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr virtual_terrain_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Variables
  cv::Mat terrain_base_;
  std::string csv_filename_;
};
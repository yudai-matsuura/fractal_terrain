#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

struct PointXYZ
{
    float x;
    float y;
    float z;
}

class FractalTerrain : public rclcpp::Node
{
public:
    FractalTerrain() : Node("fractal_terrain")

private:
  /**
   * @brief
   *
   * @param start
   * @param end
   * @param num
   */
    std::vector<double> linsoace(double start, double end, int num);

	/**
   * @brief
   *
   * @param size
   * @param omega
   * @param target_std
   */
	cv::Mat generateFractalTerrain(int size, float omega, float target_std);

	/**
   * @brief
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
	void timer_callback();

	/**
   * @brief
   *
   * @param base_terrain
   */
  sensor_msgs::msg::PointCloud2 createPointCloudMsg(const cv::Mat & base_terrain);

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr2 virtual_terrain_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Variables
  cv::Mat terrain_base_;
  std::string csv_filename_;
}
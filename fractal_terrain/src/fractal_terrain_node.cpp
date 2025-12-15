#include "fractal_terrain/fractal_terrain_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FractalTerrainPublisher>());
  rclcpp::shutdown();
  return 0;
}

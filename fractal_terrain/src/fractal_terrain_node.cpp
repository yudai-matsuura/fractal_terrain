#include "fractal_terrain/fractal_terrain.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<FractalTerrain>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

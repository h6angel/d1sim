#include "gazebo_obstacle_cloud/obstacle_cloud_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gazebo_obstacle_cloud::ObstacleCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

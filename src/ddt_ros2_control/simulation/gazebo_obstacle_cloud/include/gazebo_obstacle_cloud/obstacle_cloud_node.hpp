#ifndef GAZEBO_OBSTACLE_CLOUD__OBSTACLE_CLOUD_NODE_HPP_
#define GAZEBO_OBSTACLE_CLOUD__OBSTACLE_CLOUD_NODE_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace gazebo_obstacle_cloud
{

class ObstacleCloudNode : public rclcpp::Node
{
public:
  explicit ObstacleCloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
  void publishCachedMessages();
  sensor_msgs::msg::PointCloud2 buildObstacleCloud(
    const gazebo_msgs::msg::ModelStates & msg, const rclcpp::Time & stamp) const;
  void sampleAxisAlignedBox(
    const geometry_msgs::msg::Pose & pose, std::vector<float> & points) const;

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;

  sensor_msgs::msg::PointCloud2 cached_cloud_;
  nav_msgs::msg::Odometry cached_odom_;
  bool has_cached_cloud_{false};
  bool has_cached_odom_{false};

  std::string model_states_topic_;
  std::string obstacle_name_prefix_;
  std::string robot_model_name_;
  std::string frame_id_;
  std::string child_frame_id_;
  double box_size_x_{1.0};
  double box_size_y_{1.0};
  double box_size_z_{1.0};
  double sample_step_{0.1};
};

}  // namespace gazebo_obstacle_cloud

#endif  // GAZEBO_OBSTACLE_CLOUD__OBSTACLE_CLOUD_NODE_HPP_

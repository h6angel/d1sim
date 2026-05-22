#include "gazebo_obstacle_cloud/obstacle_cloud_node.hpp"

#include <algorithm>
#include <chrono>
#include <regex>

namespace gazebo_obstacle_cloud
{

namespace
{
constexpr double kEpsilon = 1e-6;
}

ObstacleCloudNode::ObstacleCloudNode(const rclcpp::NodeOptions & options)
: Node("obstacle_cloud_node", options)
{
  model_states_topic_ = this->declare_parameter<std::string>("model_states_topic", "/model_states");
  const std::string cloud_topic = this->declare_parameter<std::string>("cloud_topic", "/gazebo_obstacles");
  const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom");
  obstacle_name_prefix_ =
    this->declare_parameter<std::string>("obstacle_name_prefix", "obstacle_");
  robot_model_name_ = this->declare_parameter<std::string>("robot_model_name", "d1h_description");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "world");
  child_frame_id_ = this->declare_parameter<std::string>("child_frame_id", "base_link");
  box_size_x_ = this->declare_parameter<double>("box_size_x", 1.0);
  box_size_y_ = this->declare_parameter<double>("box_size_y", 1.0);
  box_size_z_ = this->declare_parameter<double>("box_size_z", 1.0);
  sample_step_ = this->declare_parameter<double>("sample_step", 0.1);
  const double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 10.0);

  if (sample_step_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "sample_step must be positive, resetting to 0.1");
    sample_step_ = 0.1;
  }

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

  model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
    model_states_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ObstacleCloudNode::modelStatesCallback, this, std::placeholders::_1));

  if (publish_rate_hz > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    republish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ObstacleCloudNode::publishCachedMessages, this));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Listening on %s, publishing %s and %s (robot: %s, prefix: %s)",
    model_states_topic_.c_str(), cloud_topic.c_str(), odom_topic.c_str(),
    robot_model_name_.c_str(), obstacle_name_prefix_.c_str());
}

bool ObstacleCloudNode::resolveBoxSize(
  const std::string & model_name, double & size_x, double & size_y, double & size_z) const
{
  // obstacle_<sx>_<sy>_<sz>_... with sizes in decimeters (e.g. 240 -> 24.0 m)
  static const std::regex size_re(R"(^obstacle_(\d+)_(\d+)_(\d+)_)");
  std::smatch match;
  if (std::regex_search(model_name, match, size_re)) {
    size_x = std::stod(match[1].str()) * 0.1;
    size_y = std::stod(match[2].str()) * 0.1;
    size_z = std::stod(match[3].str()) * 0.1;
    return true;
  }
  size_x = box_size_x_;
  size_y = box_size_y_;
  size_z = box_size_z_;
  return false;
}

void ObstacleCloudNode::sampleAxisAlignedBox(
  const geometry_msgs::msg::Pose & pose, double size_x, double size_y, double size_z,
  std::vector<float> & points) const
{
  const double hx = size_x * 0.5;
  const double hy = size_y * 0.5;
  const double hz = size_z * 0.5;
  const double cx = pose.position.x;
  const double cy = pose.position.y;
  const double cz = pose.position.z;

  for (double x = cx - hx; x <= cx + hx + kEpsilon; x += sample_step_) {
    for (double y = cy - hy; y <= cy + hy + kEpsilon; y += sample_step_) {
      for (double z = cz - hz; z <= cz + hz + kEpsilon; z += sample_step_) {
        points.push_back(static_cast<float>(x));
        points.push_back(static_cast<float>(y));
        points.push_back(static_cast<float>(z));
      }
    }
  }
}

sensor_msgs::msg::PointCloud2 ObstacleCloudNode::buildObstacleCloud(
  const gazebo_msgs::msg::ModelStates & msg, const rclcpp::Time & stamp) const
{
  std::vector<float> xyz;
  xyz.reserve(msg.name.size() * 1500);

  for (size_t i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i].rfind(obstacle_name_prefix_, 0) != 0) {
      continue;
    }
    double sx = box_size_x_;
    double sy = box_size_y_;
    double sz = box_size_z_;
    resolveBoxSize(msg.name[i], sx, sy, sz);
    sampleAxisAlignedBox(msg.pose[i], sx, sy, sz, xyz);
  }

  const size_t point_count = xyz.size() / 3;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id_;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(point_count);
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = xyz[3 * i];
    *iter_y = xyz[3 * i + 1];
    *iter_z = xyz[3 * i + 2];
  }

  return cloud;
}

void ObstacleCloudNode::modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  const rclcpp::Time stamp = this->now();

  cached_cloud_ = buildObstacleCloud(*msg, stamp);
  has_cached_cloud_ = true;

  const auto robot_it = std::find(msg->name.begin(), msg->name.end(), robot_model_name_);
  if (robot_it != msg->name.end()) {
    const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), robot_it));
    cached_odom_.header.stamp = stamp;
    cached_odom_.header.frame_id = frame_id_;
    cached_odom_.child_frame_id = child_frame_id_;
    cached_odom_.pose.pose = msg->pose[idx];
    cached_odom_.twist.twist = msg->twist[idx];
    has_cached_odom_ = true;
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Robot model '%s' not found in model_states", robot_model_name_.c_str());
  }

  publishCachedMessages();
}

void ObstacleCloudNode::publishCachedMessages()
{
  const auto stamp = this->now();

  if (has_cached_cloud_) {
    cached_cloud_.header.stamp = stamp;
    cloud_pub_->publish(cached_cloud_);
  }

  if (has_cached_odom_) {
    cached_odom_.header.stamp = stamp;
    odom_pub_->publish(cached_odom_);
  }
}

}  // namespace gazebo_obstacle_cloud

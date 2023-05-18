#include <Eigen/Dense>
#include <filesystem>
#include <regex>

#include "gnss_reset/embed_gnss2map.hpp"

using namespace std::chrono_literals;

namespace embed_gnss2map
{
EmbedGnss2MapNode::EmbedGnss2MapNode() : Node("embed_gnss2map_node"), get_robot_pose_(false)
{
  writer_ = std::make_shared<rosbag2_cpp::Writer>();
  initPubSub();
  initTimer();
  setParam();
  getParam();
}

void EmbedGnss2MapNode::initPubSub()
{
  pub_map_with_gnss_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map_with_gnss", 1);
  pub_debug_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("debug_map", 1);

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
  sub_gnss_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 10, std::bind(&EmbedGnss2MapNode::gnssCb, this, std::placeholders::_1));
  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", qos_profile, std::bind(&EmbedGnss2MapNode::mapCb, this, std::placeholders::_1));
  sub_robot_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1, std::bind(&EmbedGnss2MapNode::getRobotPose, this, std::placeholders::_1));
}

void EmbedGnss2MapNode::initTimer()
{
  debug_timer_ = create_wall_timer(
      100ms, std::bind(&EmbedGnss2MapNode::getMapIndexFromRobotPoseDebugTimerCb, this));
}

void EmbedGnss2MapNode::setParam() { declare_parameter("output_rosbag_path", "map_with_gnss"); }

void EmbedGnss2MapNode::getParam()
{
  output_rosbag_path_ = get_parameter("output_rosbag_path").as_string();
}

void EmbedGnss2MapNode::gnssCb(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (get_robot_pose_)
  {
    gnss_ = *msg;
    auto index = getMapIndexFromRobotPose();
    embedGnss2Map(index);
  }
}

void EmbedGnss2MapNode::getMapIndexFromRobotPoseDebugTimerCb()
{
  if (get_robot_pose_)
  {
    auto index = getMapIndexFromRobotPose();
    getMapIndexFromRobotPoseDebug();
    //   embedGnss2Map(index);
  }
}

void EmbedGnss2MapNode::getRobotPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  current_robot_pose_.header = msg->header;
  current_robot_pose_.pose = msg->pose.pose;
  get_robot_pose_ = true;
}

uint64_t EmbedGnss2MapNode::getMapIndexFromRobotPose()
{
  int grid_x = current_robot_pose_.pose.position.x / map_.info.resolution;
  int grid_y = current_robot_pose_.pose.position.y / map_.info.resolution;

  return grid_y * map_.info.width + grid_x;
}

void EmbedGnss2MapNode::getMapIndexFromRobotPoseDebug()
{
  auto map = map_;

  Eigen::Vector2d target(current_robot_pose_.pose.position.x, current_robot_pose_.pose.position.y);

  for (auto& data : map.data) data = 1;

  for (auto y = 0; y < map_.info.height; ++y)
    for (auto x = 0; x < map_.info.width; ++x)
    {
      Eigen::Vector2d reference(x * map_.info.resolution, y * map_.info.resolution);
      auto l2_norm = (target - reference).norm();

      if (l2_norm < 0.5)
      {
        auto index = y * map_.info.width + x;
        map.data[index] = 99;
      }
    }

  pub_debug_map_->publish(map);
}

void EmbedGnss2MapNode::embedGnss2Map(uint64_t index) { map_with_gnss_.gnss_data[index] = gnss_; }

void EmbedGnss2MapNode::mapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  map_ = *msg;

  map_with_gnss_.header = msg->header;
  map_with_gnss_.info = msg->info;
  int map_size = msg->info.width * msg->info.height;
  map_with_gnss_.data.resize(map_size);
  std::copy(std::begin(msg->data), std::end(msg->data), std::begin(map_with_gnss_.data));

  // publishMapWithGnss();
  // writeRosbag();
}

void EmbedGnss2MapNode::publishMapWithGnss()
{
  auto occupancy_grid = nav_msgs::msg::OccupancyGrid();

  occupancy_grid.header.stamp = now();
  occupancy_grid.header = map_with_gnss_.header;
  occupancy_grid.info.map_load_time = now();
  occupancy_grid.info = map_with_gnss_.info;

  int map_size = map_with_gnss_.info.width * map_with_gnss_.info.height;
  occupancy_grid.data.resize(map_size);

  std::copy(std::begin(map_with_gnss_.data), std::end(map_with_gnss_.data),
            std::begin(occupancy_grid.data));

  pub_map_with_gnss_->publish(occupancy_grid);
}

void EmbedGnss2MapNode::writeRosbag()
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_rosbag_path_;
  storage_options.storage_id = "sqlite3";

  try
  {
    if (not isDirectoryPresent(output_rosbag_path_))
      writer_->open(storage_options);
    else
      writer_->open(avoidDirectoryNameCollision(output_rosbag_path_));
  }
  catch (const std::exception& e)
  {
    std::cerr << "Caught an exception: " << e.what() << '\n';
  }

  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = pub_map_with_gnss_->get_topic_name();
  topic_metadata.type = "gnss_reset_msgs/msg/OccupancyGridWithNavSatFix";
  topic_metadata.serialization_format = "cdr";
  writer_->create_topic(topic_metadata);

  auto message = map_with_gnss_;
  auto timestamp = rclcpp::Time(message.header.stamp);
  writer_->write(message, topic_metadata.name, timestamp);
}

std::string EmbedGnss2MapNode::avoidDirectoryNameCollision(std::string& output_rosbag_path)
{
  int index = -1;
  while (isDirectoryPresent(output_rosbag_path))
  {
    std::regex pattern(".*_(\\d+)$");
    std::smatch match;
    if (std::regex_match(output_rosbag_path, pattern))
    {
      std::regex pattern("_(\\d+)$");
      std::regex_search(output_rosbag_path, match, pattern);
      index = std::stoi(match[1].str());
      output_rosbag_path = std::regex_replace(output_rosbag_path, pattern, "");
    }

    ++index;
    output_rosbag_path = output_rosbag_path + "_" + std::to_string(index);
  }
  return output_rosbag_path;
}

bool EmbedGnss2MapNode::isDirectoryPresent(std::string& dir_path_str)
{
  std::filesystem::path dir_path(dir_path_str);

  if (std::filesystem::exists(dir_path) && std::filesystem::is_directory(dir_path)) return true;

  return false;
}

}  // namespace embed_gnss2map

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embed_gnss2map::EmbedGnss2MapNode>());
  rclcpp::shutdown();
  return 0;
}
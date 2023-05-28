// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "gnss_reset/embed_gnss2map.hpp"

#include <Eigen/Dense>
#include <filesystem>
#include <regex>

using namespace std::chrono_literals;

namespace embed_gnss2map
{
EmbedGnss2MapNode::EmbedGnss2MapNode()
: Node("embed_gnss2map_node"), get_robot_pose_(false), get_map_(false)
{
  writer_ = std::make_shared<rosbag2_cpp::Writer>();
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  initPubSub();
  initTf();
  setParam();
  getParam();
}

EmbedGnss2MapNode::~EmbedGnss2MapNode()
{
  publishMapWithGnss();
  writeRosbag();
}

void EmbedGnss2MapNode::initPubSub()
{
  pub_map_with_gnss_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map_with_gnss", 1);
  pub_debug_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("debug_map", 1);

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
  sub_gnss_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "gnss/fix", 10, std::bind(&EmbedGnss2MapNode::gnssCb, this, std::placeholders::_1));
  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", qos_profile, std::bind(&EmbedGnss2MapNode::mapCb, this, std::placeholders::_1));
}

void EmbedGnss2MapNode::initTf()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void EmbedGnss2MapNode::setParam() { declare_parameter("output_rosbag_path", "map_with_gnss"); }

void EmbedGnss2MapNode::getParam()
{
  output_rosbag_path_ = get_parameter("output_rosbag_path").as_string();
}

void EmbedGnss2MapNode::gnssCb(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  getRobotPose(current_robot_pose_);
  if (get_robot_pose_ && get_map_) {
    gnss_ = *msg;
    auto index = getMapIndexFromRobotPose();
    getMapIndexFromRobotPoseDebug(index);
    embedGnss2Map(index);
  }
}

void EmbedGnss2MapNode::getRobotPose(geometry_msgs::msg::PoseStamped & current_robot_pose)
{
  while (rclcpp::ok() && not tf_buffer_->canTransform("map", "base_footprint", tf2::TimePoint())) {
    RCLCPP_WARN(get_logger(), "Wait Can Transform");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped = tf_buffer_->lookupTransform(
      "map", "base_footprint", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
  }

  current_robot_pose.header = transform_stamped.header;
  current_robot_pose.pose.position.x = transform_stamped.transform.translation.x;
  current_robot_pose.pose.position.y = transform_stamped.transform.translation.y;
  current_robot_pose.pose.orientation = transform_stamped.transform.rotation;

  get_robot_pose_ = true;
}

uint64_t EmbedGnss2MapNode::getMapIndexFromRobotPose()
{
  int grid_x = current_robot_pose_.pose.position.x / map_.info.resolution;
  int grid_y = current_robot_pose_.pose.position.y / map_.info.resolution;

  return grid_y * map_.info.width + grid_x;
}

void EmbedGnss2MapNode::getMapIndexFromRobotPoseDebug(int index)
{
  static bool once_flag = true;

  if (once_flag) {
    for (auto & data : map_.data) data = 1;
    once_flag = false;
  }

  map_.data[index] = 99;

  pub_debug_map_->publish(map_);
}

void EmbedGnss2MapNode::embedGnss2Map(uint64_t index) { map_with_gnss_.gnss_data[index] = gnss_; }

void EmbedGnss2MapNode::mapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  map_ = *msg;

  map_with_gnss_.header = msg->header;
  map_with_gnss_.info = msg->info;
  int map_size = msg->info.width * msg->info.height;
  map_with_gnss_.data.resize(map_size);
  map_with_gnss_.gnss_data.resize(map_size);
  std::copy(std::begin(msg->data), std::end(msg->data), std::begin(map_with_gnss_.data));

  get_map_ = true;
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

  std::copy(
    std::begin(map_with_gnss_.data), std::end(map_with_gnss_.data),
    std::begin(occupancy_grid.data));

  pub_map_with_gnss_->publish(occupancy_grid);
}

void EmbedGnss2MapNode::writeRosbag()
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_rosbag_path_;
  storage_options.storage_id = "sqlite3";

  try {
    if (not isDirectoryPresent(output_rosbag_path_))
      writer_->open(storage_options);
    else
      writer_->open(avoidDirectoryNameCollision(output_rosbag_path_));
  } catch (const std::exception & e) {
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

std::string EmbedGnss2MapNode::avoidDirectoryNameCollision(std::string & output_rosbag_path)
{
  int index = -1;
  while (isDirectoryPresent(output_rosbag_path)) {
    std::regex pattern(".*_(\\d+)$");
    std::smatch match;
    if (std::regex_match(output_rosbag_path, pattern)) {
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

bool EmbedGnss2MapNode::isDirectoryPresent(std::string & dir_path_str)
{
  std::filesystem::path dir_path(dir_path_str);

  if (std::filesystem::exists(dir_path) && std::filesystem::is_directory(dir_path)) return true;

  return false;
}

}  // namespace embed_gnss2map

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embed_gnss2map::EmbedGnss2MapNode>());
  rclcpp::shutdown();
  return 0;
}
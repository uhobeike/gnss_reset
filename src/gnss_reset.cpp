// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "gnss_reset/gnss_reset.hpp"

#include <Eigen/Dense>

#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

using namespace std::chrono_literals;

namespace gnss_reset
{
GnssResetNode::GnssResetNode() : Node("gnss_reset_node"), get_map_(false)
{
  reader_ = std::make_shared<rosbag2_cpp::Reader>();
  setParam();
  getParam();
  initPubSub();
  initTimer();
  readRosbag();
}

void GnssResetNode::initPubSub()
{
  pub_map_with_gnss_ = create_publisher<nav_msgs::msg::OccupancyGrid>("rosbag_to_map_with_gnss", 1);
  pub_debug_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("debug_reset", 1);

  sub_gnss_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "gnss/fix", 10, std::bind(&GnssResetNode::gnssCb, this, std::placeholders::_1));
}

void GnssResetNode::initTimer()
{
  timer_ = create_wall_timer(1s, std::bind(&GnssResetNode::publishMapWithGnss, this));
}

void GnssResetNode::setParam() { declare_parameter("read_rosbag_path", "map_with_gnss"); }

void GnssResetNode::getParam()
{
  read_rosbag_path_ = get_parameter("read_rosbag_path").as_string();
}

void GnssResetNode::gnssCb(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (get_map_) {
    static bool once_flag = true;
    if (once_flag) {
      publishMapWithGnss();
      once_flag = false;
    }
    gnssReset(*msg);
  }
}

void GnssResetNode::gnssReset(sensor_msgs::msg::NavSatFix msg)
{
  auto index = findNearestLatLong(msg);
  debugIndexToMap(index);
}

void GnssResetNode::debugIndexToMap(int index)
{
  auto map = map_;
  for (auto & data : map.data) data = 1;

  map.data[index] = 99;

  pub_debug_map_->publish(map);
}

unsigned int GnssResetNode::findNearestLatLong(sensor_msgs::msg::NavSatFix target_gnss_data)
{
  Eigen::Vector2d target_pose_mat(target_gnss_data.latitude, target_gnss_data.longitude);

  std::vector<double> l2_norm_vec;
  for (auto gnss_data : map_index_and_gnss_data_from_rosbag_) {
    Eigen::Vector2d reference_mat(gnss_data.second.latitude, gnss_data.second.longitude);
    l2_norm_vec.push_back((target_pose_mat - reference_mat).norm());
  }

  auto min_element_iter = std::min_element(l2_norm_vec.begin(), l2_norm_vec.end());
  unsigned int min_index = std::distance(l2_norm_vec.begin(), min_element_iter);

  return map_index_and_gnss_data_from_rosbag_[min_index].first;
}

void GnssResetNode::readRosbag()
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = read_rosbag_path_;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  reader_->open(storage_options, converter_options);

  rosbag2_cpp::SerializationFormatConverterFactory factory;
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
    cdr_deserializer;
  cdr_deserializer = factory.load_deserializer("cdr");

  auto library = rosbag2_cpp::get_typesupport_library(
    "gnss_reset_msgs/msg/OccupancyGridWithNavSatFix", "rosidl_typesupport_cpp");
  auto type_support = rosbag2_cpp::get_typesupport_handle(
    "gnss_reset_msgs/msg/OccupancyGridWithNavSatFix", "rosidl_typesupport_cpp", library);
  auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();

  while (reader_->has_next()) {
    auto serialized_message = reader_->read_next();
    if (std::strcmp(serialized_message->topic_name.c_str(), "/map_with_gnss") == 0) {
      gnss_reset_msgs::msg::OccupancyGridWithNavSatFix map_with_gnss;
      ros_message->message = &map_with_gnss;
      cdr_deserializer->deserialize(serialized_message, type_support, ros_message);

      map_with_gnss_ = map_with_gnss;
    }
  }

  auto index = 0;
  for (auto & gnss : map_with_gnss_.gnss_data) {
    if (gnss.header.stamp.sec != 0) {
      std::pair<unsigned int, sensor_msgs::msg::NavSatFix> map_index_and_gnss_data;
      map_index_and_gnss_data.first = index;
      map_index_and_gnss_data.second = gnss;
      map_index_and_gnss_data_from_rosbag_.push_back(map_index_and_gnss_data);
    }
    ++index;
  }
  get_map_ = true;
}

void GnssResetNode::publishMapWithGnss()
{
  auto occupancy_grid = nav_msgs::msg::OccupancyGrid();

  occupancy_grid.header = map_with_gnss_.header;
  occupancy_grid.info = map_with_gnss_.info;
  occupancy_grid.header.stamp = now();
  occupancy_grid.info.map_load_time = now();

  int map_size = map_with_gnss_.info.width * map_with_gnss_.info.height;
  occupancy_grid.data.resize(map_size);

  std::copy(
    std::begin(map_with_gnss_.data), std::end(map_with_gnss_.data),
    std::begin(occupancy_grid.data));

  map_ = occupancy_grid;
  pub_map_with_gnss_->publish(occupancy_grid);
}

}  // namespace gnss_reset

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_reset::GnssResetNode>());
  rclcpp::shutdown();
  return 0;
}
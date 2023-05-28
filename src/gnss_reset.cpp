// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "gnss_reset/gnss_reset.hpp"

#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

using namespace std::chrono_literals;

namespace gnss_reset
{
GnssResetNode::GnssResetNode() : Node("gnss_reset_node")
{
  reader_ = std::make_shared<rosbag2_cpp::Reader>();
  setParam();
  getParam();
  initPublisher();
  initTimer();
  readRosbag();
}

void GnssResetNode::initPublisher()
{
  pub_map_with_gnss_ = create_publisher<nav_msgs::msg::OccupancyGrid>("rosbag_to_map_with_gnss", 1);
}

void GnssResetNode::setParam() { declare_parameter("read_rosbag_path", "map_with_gnss"); }

void GnssResetNode::getParam()
{
  read_rosbag_path_ = get_parameter("read_rosbag_path").as_string();
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

  pub_map_with_gnss_->publish(occupancy_grid);
}

void GnssResetNode::on_timer() { publishMapWithGnss(); }

void GnssResetNode::initTimer()
{
  timer_ = create_wall_timer(1000ms, std::bind(&GnssResetNode::on_timer, this));
}

}  // namespace gnss_reset

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_reset::GnssResetNode>());
  rclcpp::shutdown();
  return 0;
}
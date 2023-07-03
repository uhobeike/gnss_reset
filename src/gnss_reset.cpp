// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "gnss_reset/gnss_reset.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

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

void GnssResetNode::initTf()
{
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface(),
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  latest_tf_ = tf2::Transform::getIdentity();
}

void GnssResetNode::setParam() { declare_parameter("read_rosbag_path", "map_with_gnss"); }

void GnssResetNode::getParam()
{
  read_rosbag_path_ = get_parameter("read_rosbag_path").as_string();
}

void GnssResetNode::gnssCb(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  gnss_stamp_.stamp = msg->header.stamp;
  if (get_map_) {
    static bool once_flag = true;
    if (once_flag) {
      publishMapWithGnss();
      initTf();
      once_flag = false;
    }
    gnssReset(*msg);
  }
}

void GnssResetNode::gnssReset(sensor_msgs::msg::NavSatFix msg)
{
  auto index = findNearestLatLong(msg);
  debugIndexToMap(index);
  odom2map();
}

void GnssResetNode::debugIndexToMap(int index)
{
  auto map = map_;
  for (auto & data : map.data) data = 1;

  map.data[index] = 99;

  pub_debug_map_->publish(map);
}

void GnssResetNode::odom2map()
{
  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    tf2::Transform tmp_tf(q, tf2::Vector3(9.45171, 11.5139, 0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = "base_footprint";
    tmp_tf_stamped.header.stamp = gnss_stamp_.stamp;

    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, "odom");
  } catch (tf2::TransformException & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return;
  }

  auto stamp = tf2_ros::fromMsg(gnss_stamp_.stamp);
  tf2::TimePoint transform_tolerance_ = stamp + tf2::durationFromSec(1.0);

  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = "map";
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_tolerance_);
  tmp_tf_stamped.child_frame_id = "odom";
  tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
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
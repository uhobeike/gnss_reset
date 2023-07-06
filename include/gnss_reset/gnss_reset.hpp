// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS_RESET__GNSS_RESET_HPP_
#define GNSS_RESET__GNSS_RESET_HPP_

#include "gnss_reset_msgs/msg/occupancy_grid_with_nav_sat_fix.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace gnss_reset
{

class GnssResetNode : public rclcpp::Node
{
public:
  GnssResetNode();

protected:
  void initPubSub();
  void initTimer();
  void initTf();
  void setParam();
  void getParam();

  void gnssCb(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  void readRosbag();
  void publishMapWithGnss();
  void odom2map(int index);

  void gnssReset(sensor_msgs::msg::NavSatFix msg);
  unsigned int findNearestLatLong(sensor_msgs::msg::NavSatFix target_gnss_data);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_with_gnss_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_debug_map_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform latest_tf_;

  std::shared_ptr<rosbag2_cpp::Reader> reader_;

  gnss_reset_msgs::msg::OccupancyGridWithNavSatFix map_with_gnss_;
  std::vector<std::pair<unsigned int, sensor_msgs::msg::NavSatFix>>
    map_index_and_gnss_data_from_rosbag_;
  nav_msgs::msg::OccupancyGrid map_;

  std::string read_rosbag_path_;
  bool get_map_;
  std_msgs::msg::Header gnss_stamp_;

  std::map<std::string, std::pair<double, double>> index_to_pose_;
};

}  // namespace gnss_reset

#endif  // GNSS_RESET__GNSS_RESET_HPP_

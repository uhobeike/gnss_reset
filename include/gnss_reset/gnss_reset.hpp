// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS_RESET__GNSS_RESET_HPP_
#define GNSS_RESET__GNSS_RESET_HPP_

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"

#include "gnss_reset_msgs/msg/occupancy_grid_with_nav_sat_fix.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace gnss_reset
{

class GnssResetNode : public rclcpp::Node
{
 public:
  GnssResetNode();

 protected:
  void initPublisher();
  void initTimer();
  void on_timer();
  void setParam();
  void getParam();

  void readRosbag();
  void publishMapWithGnss();

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_with_gnss_;

  std::shared_ptr<rosbag2_cpp::Reader> reader_;
  rclcpp::TimerBase::SharedPtr timer_;

  gnss_reset_msgs::msg::OccupancyGridWithNavSatFix map_with_gnss_;

  std::string read_rosbag_path_;
};

}  // namespace gnss_reset

#endif  // GNSS_RESET__GNSS_RESET_HPP_

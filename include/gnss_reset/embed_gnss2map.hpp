// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS_RESET__EMBED_GNSS2MAP_HPP_
#define GNSS_RESET__EMBED_GNSS2MAP_HPP_

#include "rclcpp/rclcpp.hpp"

#include "gnss_reset_msgs/msg/occupancy_grid_with_nav_sat_fix.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rosbag2_cpp/writer.hpp"

namespace embed_gnss2map
{

class EmbedGnss2MapNode : public rclcpp::Node
{
 public:
  EmbedGnss2MapNode();

 protected:
  void initPubSub();
  void setParam();
  void getParam();

  void embedGnss2Map();

  void mapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void publishMapWithGnss();
  void writeRosbag();
  std::string avoidDirectoryNameCollision(std::string& output_rosbag_path);
  bool isDirectoryPresent(std::string& dir_path);

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_with_gnss_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;

  std::shared_ptr<rosbag2_cpp::Writer> writer_;

  gnss_reset_msgs::msg::OccupancyGridWithNavSatFix map_with_gnss_;

  std::string output_rosbag_path_;
};

}  // namespace embed_gnss2map

#endif  // GNSS_RESET__EMBED_GNSS2MAP_HPP_

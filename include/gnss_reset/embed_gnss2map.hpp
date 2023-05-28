// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS_RESET__EMBED_GNSS2MAP_HPP_
#define GNSS_RESET__EMBED_GNSS2MAP_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gnss_reset_msgs/msg/occupancy_grid_with_nav_sat_fix.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace embed_gnss2map
{

class EmbedGnss2MapNode : public rclcpp::Node
{
public:
  EmbedGnss2MapNode();

protected:
  void initPubSub();
  void initTimer();
  void initTf();
  void setParam();
  void getParam();

  void getRobotPose(geometry_msgs::msg::PoseStamped & current_robot_pose);

  uint64_t getMapIndexFromRobotPose();
  void getMapIndexFromRobotPoseDebug();
  void getMapIndexFromRobotPoseDebugTimerCb();
  void embedGnss2Map(uint64_t index);

  void gnssCb(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void mapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void publishMapWithGnss();
  void writeRosbag();
  std::string avoidDirectoryNameCollision(std::string & output_rosbag_path);
  bool isDirectoryPresent(std::string & dir_path);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_with_gnss_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_debug_map_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_robot_pose_;

  rclcpp::TimerBase::SharedPtr debug_timer_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;

  sensor_msgs::msg::NavSatFix gnss_;
  geometry_msgs::msg::PoseStamped current_robot_pose_;
  gnss_reset_msgs::msg::OccupancyGridWithNavSatFix map_with_gnss_;
  nav_msgs::msg::OccupancyGrid map_;

  bool get_robot_pose_;
  std::string output_rosbag_path_;
};

}  // namespace embed_gnss2map

#endif  // GNSS_RESET__EMBED_GNSS2MAP_HPP_

#include "gnss_reset/embed_gnss2map.hpp"

namespace embed_gnss2map
{
EmbedGnss2MapNode::EmbedGnss2MapNode() : Node("embed_gnss2map_node") { initPubSub(); }

void EmbedGnss2MapNode::initPubSub()
{
  pub_map_with_gnss_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map_with_gnss", 1);

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", qos_profile, std::bind(&EmbedGnss2MapNode::mapCb, this, std::placeholders::_1));
}

void EmbedGnss2MapNode::embedGnss2Map() {}

void EmbedGnss2MapNode::mapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  map_with_gnss_.header = msg->header;
  map_with_gnss_.info = msg->info;
  int map_size = msg->info.width * msg->info.height;
  map_with_gnss_.data.resize(map_size);
  std::copy(std::begin(msg->data), std::end(msg->data), std::begin(map_with_gnss_.data));

  publishMapWithGnss();
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

}  // namespace embed_gnss2map

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embed_gnss2map::EmbedGnss2MapNode>());
  rclcpp::shutdown();
  return 0;
}
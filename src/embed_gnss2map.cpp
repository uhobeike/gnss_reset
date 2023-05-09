#include "gnss_reset/embed_gnss2map.hpp"

namespace embed_gnss2map
{
EmbedGnss2MapNode::EmbedGnss2MapNode() : Node("embed_gnss2map_node")
{
  initPublisher();
  publishMapWithGnss();
}

void EmbedGnss2MapNode::initPublisher()
{
  pub_map_with_gnss_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map_with_gnss", 1);
}

void EmbedGnss2MapNode::embedGnss2Map() {}

void EmbedGnss2MapNode::publishMapWithGnss()
{
  auto occupancy_grid = nav_msgs::msg::OccupancyGrid();

  occupancy_grid.header.stamp = now();
  occupancy_grid.header.frame_id = "map";

  occupancy_grid.info.map_load_time = now();
  occupancy_grid.info.resolution = 0.05;
  occupancy_grid.info.width = 100;
  occupancy_grid.info.height = 100;

  occupancy_grid.info.origin.position.x = 0.0;
  occupancy_grid.info.origin.position.y = 0.0;
  occupancy_grid.info.origin.position.z = 0.0;
  occupancy_grid.info.origin.orientation.x = 0.0;
  occupancy_grid.info.origin.orientation.y = 0.0;
  occupancy_grid.info.origin.orientation.z = 0.0;
  occupancy_grid.info.origin.orientation.w = 1.0;

  int map_size = occupancy_grid.info.width * occupancy_grid.info.height;
  occupancy_grid.data.resize(map_size);
  map_with_gnss_.data.resize(map_size);

  for (uint32_t row = 0; row < occupancy_grid.info.height; ++row)
    for (uint32_t col = 0; col < occupancy_grid.info.width; ++col)
      map_with_gnss_.data[row * occupancy_grid.info.width + col] = 0;

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
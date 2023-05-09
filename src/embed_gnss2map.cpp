#include "gnss_reset/embed_gnss2map.hpp"

namespace embed_gnss2map
{
EmbedGnss2MapNode::EmbedGnss2MapNode() : Node("embed_gnss2map_node") {}
}  // namespace embed_gnss2map

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embed_gnss2map::EmbedGnss2MapNode>());
  rclcpp::shutdown();
  return 0;
}
// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "gnss_reset/gnss_reset.hpp"

namespace gnss_reset
{
GnssResetNode::GnssResetNode() : Node("gnss_reset_node") {}
}  // namespace gnss_reset

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_reset::GnssResetNode>());
  rclcpp::shutdown();
  return 0;
}
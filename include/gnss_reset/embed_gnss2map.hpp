// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS_RESET__EMBED_GNSS2MAP_HPP_
#define GNSS_RESET__EMBED_GNSS2MAP_HPP_

#include "rclcpp/rclcpp.hpp"

namespace embed_gnss2map
{

class EmbedGnss2MapNode : public rclcpp::Node
{
 public:
  EmbedGnss2MapNode();

 protected:
 private:
};

}  // namespace embed_gnss2map

#endif  // GNSS_RESET__EMBED_GNSS2MAP_HPP_

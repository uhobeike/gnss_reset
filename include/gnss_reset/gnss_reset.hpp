// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS_RESET__GNSS_RESET_HPP_
#define GNSS_RESET__GNSS_RESET_HPP_

#include "rclcpp/rclcpp.hpp"

namespace gnss_reset
{

class GnssResetNode : public rclcpp::Node
{
 public:
  GnssResetNode();

 protected:
 private:
};

}  // namespace gnss_reset

#endif  // GNSS_RESET__GNSS_RESET_HPP_

#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"
#include <cmath>

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
  }
};

#endif

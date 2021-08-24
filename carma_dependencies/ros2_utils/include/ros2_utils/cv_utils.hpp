// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_UTILS__CV_UTILS_HPP_
#define ROS2_UTILS__CV_UTILS_HPP_

#include <string>

#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/time.hpp"
#include "opencv2/core/mat.hpp"

namespace ros2_utils
{

std::string mat_type2encoding(int mat_type);
int encoding2mat_type(const std::string & encoding);
sensor_msgs::msg::Image::UniquePtr toImageMsg(
  const cv::Mat & image,
  const rclcpp::Time current_time,
  const char * frame_id = "");

}  // namespace ros2_utils

#endif  // ROS2_UTILS__CV_UTILS_HPP_

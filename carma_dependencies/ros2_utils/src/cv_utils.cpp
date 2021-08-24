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

#include "ros2_utils/cv_utils.hpp"

#include <string>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

namespace ros2_utils
{

std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("unsupported encoding type");
  }
}

int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  }
  throw std::runtime_error("Unsupported mat type");
}

sensor_msgs::msg::Image::UniquePtr
toImageMsg(const cv::Mat & image, const rclcpp::Time current_time, const char * frame_id)
{
  sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

  image_msg->header.stamp = current_time;
  image_msg->header.frame_id = frame_id;
  image_msg->height = image.rows;
  image_msg->width = image.cols;
  image_msg->encoding = mat_type2encoding(image.type());
  image_msg->is_bigendian = false;
  image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
  image_msg->data.assign(image.datastart, image.dataend);

  return image_msg;
}

}  // namespace ros2_utils

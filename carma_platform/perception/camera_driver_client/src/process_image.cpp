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

#include "camera_driver_client/process_image.hpp"

#include <memory>
#include <string>

namespace process_image
{

ProcessImage::ProcessImage()
{
}

ProcessImage::ProcessImage(carma_utils::CarmaNode::SharedPtr node)
: node_(node)
{
}

void
ProcessImage::configure()
{
  RCLCPP_INFO(node_->get_logger(), "Helper class on configure");
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "camera/image", 1,
    std::bind(&ProcessImage::image_callback, this, std::placeholders::_1));
}

void
ProcessImage::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Helper class on activate");
  classification_pub_->on_activate();
}

void
ProcessImage::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), " Helper class on deactivate");
  classification_pub_->on_deactivate();
}

void
ProcessImage::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Helper class on cleanup");
  classification_pub_.reset();
}

void
ProcessImage::publish_classification()
{
  auto message = std_msgs::msg::String();
  message.data = "Vehicle Detected";
  classification_pub_->publish(message);
}

void
ProcessImage::image_callback(const sensor_msgs::msg::Image::UniquePtr msg)
{
  RCLCPP_INFO(
    node_->get_logger(), "Helper class received image in frame %s, publishing classification",
    msg->header.frame_id.c_str());
}

}  // namespace process_image

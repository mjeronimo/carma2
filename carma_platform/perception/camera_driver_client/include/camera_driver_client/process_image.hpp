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


#ifndef CAMERA_DRIVER_CLIENT__PROCESS_IMAGE_HPP_
#define CAMERA_DRIVER_CLIENT__PROCESS_IMAGE_HPP_

#include <memory>

#include "carma_utils/carma_lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/lifecycle_interface.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace process_image
{

class ProcessImage : public ros2_utils::LifecycleInterface
{
public:
  ProcessImage();
  explicit ProcessImage(carma_utils::CarmaLifecycleNode::SharedPtr node);

  void configure() override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

protected:
  void publish_classification();
  void image_callback(const sensor_msgs::msg::Image::UniquePtr msg);

  // Sample pub and sub
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Node interface
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> classification_pub_;
};

}  // namespace process_image

#endif  //  CAMERA_DRIVER_CLIENT__PROCESS_IMAGE_HPP_

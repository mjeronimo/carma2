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

#ifndef CAMERA_DRIVER_CLIENT__CAMERA_DRIVER_CLIENT_HPP_
#define CAMERA_DRIVER_CLIENT__CAMERA_DRIVER_CLIENT_HPP_

#include <string>

#include "carma_utils/carma_lifecycle_node.hpp"
#include "camera_driver_client/process_image.hpp"
#include "carma_utils/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_driver_client
{

class CameraDriverClient : public carma_utils::CarmaLifecycleNode
{
public:
  CARMA_UTILS_PUBLIC
  explicit CameraDriverClient(const rclcpp::NodeOptions & options);

protected:
  carma_utils::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg) override;

  bool show_image_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  void image_callback(const sensor_msgs::msg::Image::UniquePtr msg);

  // Helper Class
  process_image::ProcessImage image_classifier_;

  bool active_{false};
};

}  // namespace camera_driver_client

#endif  //  CAMERA_DRIVER_CLIENT__CAMERA_DRIVER_CLIENT_HPP_

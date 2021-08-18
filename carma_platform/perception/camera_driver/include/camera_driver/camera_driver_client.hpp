//
// Copyright (C) 2021 LEIDOS.
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy of
// the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.
//

#ifndef CAMERA_DRIVER__CAMERA_DRIVER_CLIENT_HPP_
#define CAMERA_DRIVER__CAMERA_DRIVER_CLIENT_HPP_

#include "carma_utils/carma_node.hpp"
#include "carma_utils/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_driver_client
{

class CameraDriverClient : public carma_utils::CarmaNode
{
public:
  CameraDriverClient();
  CARMA_UTILS_PUBLIC
  explicit CameraDriverClient(const rclcpp::NodeOptions & options);

protected:
  carma_utils::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void handle_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg) override;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
  bool show_image_{false};
};

}  // namespace camera_driver_client

#endif  //  CAMERA_DRIVER__CAMERA_DRIVER_CLIENT_HPP_

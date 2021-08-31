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

#ifndef CAMERA_DRIVER__CAMERA_DRIVER_HPP_
#define CAMERA_DRIVER__CAMERA_DRIVER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "carma_utils/carma_lifecycle_node.hpp"
#include "carma_utils/visibility_control.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_driver
{

class CameraDriver : public carma_utils::CarmaLifecycleNode
{
public:
  CARMA_UTILS_PUBLIC
  explicit CameraDriver(const rclcpp::NodeOptions & options);

protected:
  carma_utils::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg) override;

  rclcpp::TimerBase::SharedPtr timer_;

  cv::Mat image_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> image_pub_;
  void publish_image();
};

}  // namespace camera_driver

#endif  //  CAMERA_DRIVER__CAMERA_DRIVER_HPP_

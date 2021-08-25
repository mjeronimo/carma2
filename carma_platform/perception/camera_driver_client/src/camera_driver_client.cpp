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

#include "camera_driver_client/camera_driver_client.hpp"
#include <string>

#include <memory>

#include "opencv2/highgui/highgui.hpp"
#include "ros2_utils/cv_utils.hpp"

namespace camera_driver_client
{

CameraDriverClient::CameraDriverClient(const rclcpp::NodeOptions & options)
: CarmaNode(options)
{
  declare_parameter("show_image", rclcpp::ParameterValue(false));
}

carma_utils::CallbackReturn
CameraDriverClient::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  CarmaNode::on_configure(state);

  get_parameter("show_image", show_image_);
  RCLCPP_INFO(get_logger(), "Show image: %d", show_image_);

  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(
    system_alert_topic_, 1,
    std::bind(&CameraDriverClient::on_system_alert, this, std::placeholders::_1));

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "camera/image", 1,
    std::bind(&CameraDriverClient::image_callback, this, std::placeholders::_1));

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriverClient::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  CarmaNode::on_activate(state);
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriverClient::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  CarmaNode::on_deactivate(state);
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriverClient::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  CarmaNode::on_cleanup(state);
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriverClient::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriverClient::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node error");
  return carma_utils::CallbackReturn::SUCCESS;
}

void
CameraDriverClient::on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Received SystemAlert message of type: %u, msg: %s",
    msg->type, msg->description.c_str());
  RCLCPP_INFO(get_logger(), "Perform CameraDriverClient-specific system event handling");
}

void
CameraDriverClient::image_callback(const sensor_msgs::msg::Image::UniquePtr msg)
{
  RCLCPP_INFO(
    get_logger(), "received message, at address %p",
    (void *) reinterpret_cast<std::uintptr_t>(msg.get()));

  if (show_image_) {
    try {
      cv::Mat cv_mat(msg->height, msg->width,
        ros2_utils::encoding2mat_type(msg->encoding), msg->data.data());
      cv::imshow("view", cv_mat);
      cv::waitKey(10);
    } catch (std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
}

}  // namespace camera_driver_client

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver_client::CameraDriverClient)

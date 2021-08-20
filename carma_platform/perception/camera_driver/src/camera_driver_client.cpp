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

#include "camera_driver/camera_driver_client.hpp"
#include <string>

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

namespace camera_driver_client
{

CameraDriverClient::CameraDriverClient()
: CarmaNode("camera_driver_client")
{
}

CameraDriverClient::CameraDriverClient(const rclcpp::NodeOptions & options)
: CarmaNode(options)
{
}

carma_utils::CallbackReturn
CameraDriverClient::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(
    system_alert_topic_, 1,
    std::bind(&CameraDriverClient::handle_system_alert, this, std::placeholders::_1));

  cam_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "camera/image", 1,
    std::bind(&CameraDriverClient::image_callback, this, std::placeholders::_1));

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriverClient::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Create bond with the lifecycle manager
  create_bond();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriverClient::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Destroy the bond with the lifecycle manager
  destroy_bond();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriverClient::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
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
CameraDriverClient::handle_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Received SystemAlert message of type: %u, msg: %s",
    msg->type, msg->description.c_str());
  RCLCPP_INFO(get_logger(), "Perform CameraDriverClient-specific system event handling");
}

void
CameraDriverClient::image_callback(const sensor_msgs::msg::Image::UniquePtr msg)
{
  try {
    if (show_image_) {
      cv::Mat cv_mat(msg->height, msg->width, encoding2mat_type(msg->encoding), msg->data.data());
      cv::imshow("view", cv_mat);
      cv::waitKey(10);
    }
    RCLCPP_INFO(
      get_logger(), "received message, at address %p",
      (void *)reinterpret_cast<std::uintptr_t>(msg.get()));
  } catch (cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int
CameraDriverClient::encoding2mat_type(const std::string & encoding)
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

}  // namespace camera_driver_client

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver_client::CameraDriverClient)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camera_driver_client::CameraDriverClient>();
  node->spin();
  rclcpp::shutdown();

  return 0;
}

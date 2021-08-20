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

#include "camera_driver/camera_driver.hpp"

#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{

CameraDriver::CameraDriver(const rclcpp::NodeOptions & options)
: CarmaNode(options)
{
}

carma_utils::CallbackReturn
CameraDriver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("camera_driver");

  try {
    image = cv::imread(package_share_directory + "/resources/image.jpg", cv::IMREAD_COLOR);
  } catch (cv::Exception & e) {
    RCLCPP_INFO(get_logger(), "Failed to Load Image");
  }

  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(
    system_alert_topic_, 1,
    std::bind(&CameraDriver::on_system_alert, this, std::placeholders::_1));

  cam_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

  // Use a timer to schedule periodic message publishing
  // TODO: Move to on_activate (unless timer can be pause/resumed)
  timer_ = create_wall_timer(1s, std::bind(&CameraDriver::publish_image, this));

  active_ = true;
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  cam_pub_->on_activate();
  // Create bond with the lifecycle manager
  create_bond();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  cam_pub_->on_deactivate();
  // Destroy the bond with the lifecycle manager
  destroy_bond();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  cam_pub_.reset();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node error");
  return carma_utils::CallbackReturn::SUCCESS;
}

void
CameraDriver::on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Received SystemAlert message of type: %u, msg: %s",
    msg->type, msg->description.c_str());
  RCLCPP_INFO(get_logger(), "Perform CameraDriver-specific system event handling");
}

std::string
CameraDriver::mat_type2encoding(int mat_type)
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

void CameraDriver::publish_image()
{
  // Use a unique_ptr for no copy intra-process communication
  sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

  // Convert OpenCV Mat to ROS Image
  image_msg->header.stamp = this->get_clock()->now();
  image_msg->header.frame_id = "";
  image_msg->height = image.rows;
  image_msg->width = image.cols;
  image_msg->encoding = mat_type2encoding(image.type());
  image_msg->is_bigendian = false;
  image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
  image_msg->data.assign(image.datastart, image.dataend);

  if (active_) {
    if (!cam_pub_->is_activated()) {
      RCLCPP_INFO(get_logger(), "Camera Driver is currently inactive. Messages are not published.");
    } else {
      RCLCPP_INFO(
        get_logger(), "publishing image at address %p",
        (void *)reinterpret_cast<std::uintptr_t>(image_msg.get()));
      cam_pub_->publish(std::move(image_msg));
    }
  }
}

}  // namespace camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::CameraDriver)

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
#include "opencv2/highgui/highgui.hpp"
#include "ros2_utils/cv_utils.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{

CameraDriver::CameraDriver(const rclcpp::NodeOptions & options)
: CarmaNode(options)
{
}

carma_utils::CallbackReturn
CameraDriver::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  CarmaNode::on_configure(state);

  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(
    system_alert_topic_, 1,
    std::bind(&CameraDriver::on_system_alert, this, std::placeholders::_1));

  std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("camera_driver");

  try {
    image_ = cv::imread(package_share_directory + "/resources/image.jpg", cv::IMREAD_COLOR);
  } catch (cv::Exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to Load Image");
  }

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

  // Use a timer to schedule periodic message publishing
  timer_ = create_wall_timer(1s, std::bind(&CameraDriver::publish_image, this));

  // cancel the timer immediately to prevent it running the first time.
  timer_->cancel();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  timer_->reset();
  CarmaNode::on_activate(state);
  image_pub_->on_activate();

  // Use a timer to schedule periodic message publishing
  timer_ = create_wall_timer(1s, std::bind(&CameraDriver::publish_image, this));

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  timer_->cancel();
  CarmaNode::on_deactivate(state);
  image_pub_->on_deactivate();
  timer_.reset();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  CarmaNode::on_cleanup(state);
  timer_->cancel();
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

void
CameraDriver::publish_image()
{
  // Use a unique_ptr for no copy intra-process communication
  auto image_msg = ros2_utils::toImageMsg(image_, get_clock()->now(), "some_frame_id");

  RCLCPP_INFO(
    get_logger(), "publishing image at address %p",
    (void *)reinterpret_cast<std::uintptr_t>(image_msg.get()));

  image_pub_->publish(std::move(image_msg));
}

}  // namespace camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::CameraDriver)

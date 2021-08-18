// Copyright (C) 2021 LEIDOS.
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
using namespace std::chrono_literals;

namespace camera_driver
{

CameraDriver::CameraDriver()
: CarmaNode("camera_driver")
{
}

CameraDriver::CameraDriver(const rclcpp::NodeOptions & options)
: CarmaNode(options)
{
}

CameraDriver::~CameraDriver()
{
}

carma_utils::CallbackReturn
CameraDriver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("camera_driver");

  try
  {
    image = cv::imread(package_share_directory+"/resources/image.jpg", cv::IMREAD_COLOR);
  }
  catch(cv::Exception& e )
  {
    RCLCPP_INFO(get_logger(), "Failed to Load Image");
  }

  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(system_alert_topic_, 1, 
        std::bind(&CameraDriver::handle_system_alert, this, std::placeholders::_1));
  cam_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

  // Use a timer to schedule periodic message publishing
  timer_ = create_wall_timer(500ms, std::bind(&CameraDriver::publish_image, this));

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
  cam_pub_.reset();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CameraDriver::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node error");
  return carma_utils::CallbackReturn::SUCCESS;
}

void
CameraDriver::handle_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(),"Received SystemAlert message of type: %u, msg: %s",
              msg->type,msg->description.c_str());
  RCLCPP_INFO(get_logger(),"Perform CameraDriver-specific system event handling");
}

void CameraDriver::publish_image()
{
  sensor_msgs::msg::Image::SharedPtr msg;
  std_msgs::msg::Header hdr;
  msg = cv_bridge::CvImage(hdr, "bgr8", image).toImageMsg();

  if (active_) {
    if (!cam_pub_->is_activated()) {
      RCLCPP_INFO(get_logger(), "Camera Driver is currently inactive. Messages are not published.");
    } else {
      cam_pub_->publish(*msg);
    }
  }
}

}  // namespace camera_driver


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::CameraDriver)

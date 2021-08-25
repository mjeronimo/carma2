// Copyright (c) 2018 Intel Corporation
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

#ifndef ROS2_UTILS__SERVICE_CLIENT_HPP_
#define ROS2_UTILS__SERVICE_CLIENT_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/node_utils.hpp"

namespace ros2_utils
{

// A simple wrapper for a service client
template<class ServiceT>
class ServiceClient
{
public:
  explicit ServiceClient(
    const std::string & service_name,
    const rclcpp::Node::SharedPtr & provided_node)
  : service_name_(service_name), node_(provided_node)
  {
    if (provided_node) {
      node_ = provided_node;
    } else {
      node_ = generate_internal_node(service_name + "_Node");
    }
    client_ = node_->create_client<ServiceT>(service_name);
  }

  ServiceClient(const std::string & service_name, const std::string & parent_name)
  : service_name_(service_name)
  {
    node_ = generate_internal_node(parent_name + std::string("_") + service_name + "_client");
    client_ = node_->create_client<ServiceT>(service_name);
  }

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  /**
  * @brief Invoke the service and block until completed or timed out
  * @param request The request object to call the service using
  * @param timeout Maximum timeout to wait for, default infinite
  * @return Response A pointer to the service response from the request
  */
  typename ResponseType::SharedPtr invoke(
    typename RequestType::SharedPtr & request,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
  {
    // TODO(mjeronimo): overall timeout?
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future_result, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error(service_name_ + " service client: async_send_request failed");
    }

    return future_result.get();
  }

  /**
  * @brief Invoke the service and block until completed
  * @param request The request object to call the service using
  * @param Response A pointer to the service response from the request
  * @return bool Whether it was successfully called
  */
  bool invoke(
    typename RequestType::SharedPtr & request,
    typename ResponseType::SharedPtr & response)
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future_result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }

    response = future_result.get();
    return response.get();
  }

  bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
  {
    return client_->wait_for_service(timeout);
  }

protected:
  std::string service_name_;
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace ros2_utils

#endif  // ROS2_UTILS__SERVICE_CLIENT_HPP_

// Copyright (c) 2019 Intel Corporation
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

#include "ros2_utils/lifecycle_service_client.hpp"

#include <chrono>
#include <string>
#include <memory>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "ros2_utils/node_utils.hpp"

using std::chrono::seconds;
using std::chrono::high_resolution_clock;

namespace ros2_utils
{

LifecycleServiceClient::LifecycleServiceClient(const std::string & lifecycle_node_name)
: node_(generate_internal_node(lifecycle_node_name + "_lifecycle_client")),
  change_state_client_(lifecycle_node_name + "/change_state", node_),
  get_state_client_(lifecycle_node_name + "/get_state", node_)
{
}

LifecycleServiceClient::LifecycleServiceClient(
  const std::string & lifecycle_node_name,
  rclcpp::Node::SharedPtr parent_node)
: node_(parent_node),
  change_state_client_(lifecycle_node_name + "/change_state", node_),
  get_state_client_(lifecycle_node_name + "/get_state", node_)
{
}

bool
LifecycleServiceClient::change_state(const uint8_t transition, const seconds timeout)
{
  if (change_state_client_.wait_for_service(timeout)) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    change_state_client_.invoke(request, timeout);
    return true;  // TODO(mjeronimo): check the return code from invoke()
  }

  return true;
}

bool
LifecycleServiceClient::change_state(std::uint8_t transition)
{
  if (change_state_client_.wait_for_service()) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
    request->transition.id = transition;
    return change_state_client_.invoke(request, response);
  }

  return false;
}

uint8_t
LifecycleServiceClient::get_state(const seconds timeout)
{
  if (get_state_client_.wait_for_service(timeout)) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = get_state_client_.invoke(request, timeout);
    return result->current_state.id;
  }

  // TODO(mjeronimo)
  throw 0;
}

}  // namespace ros2_utils

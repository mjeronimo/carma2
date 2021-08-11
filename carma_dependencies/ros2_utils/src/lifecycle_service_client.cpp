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

using std::chrono::seconds;
using std::make_shared;
using std::string;
using std::chrono::high_resolution_clock;
using std::to_string;

namespace ros2_utils
{

std::string
sanitize_node_name(const string & potential_node_name)
{
  string node_name(potential_node_name);
  // read this as `replace` characters in `node_name` `if` not alphanumeric.
  // replace with '_'
  replace_if(
    begin(node_name), end(node_name),
    [](auto c) {return !isalnum(c);},
    '_');
  return node_name;
}

std::string time_to_string(size_t len)
{
  string output(len, '0');  // prefill the string with zeros
  auto timepoint = high_resolution_clock::now();
  auto timecount = timepoint.time_since_epoch().count();
  auto timestring = to_string(timecount);
  if (timestring.length() >= len) {
    // if `timestring` is shorter, put it at the end of `output`
    output.replace(
      0, len,
      timestring,
      timestring.length() - len, len);
  } else {
    // if `output` is shorter, just copy in the end of `timestring`
    output.replace(
      len - timestring.length(), timestring.length(),
      timestring,
      0, timestring.length());
  }
  return output;
}

std::string
generate_internal_node_name(const std::string & prefix)
{
  return sanitize_node_name(prefix) + "_" + time_to_string(8);
}

rclcpp::Node::SharedPtr
generate_internal_node(const std::string & prefix)
{
  auto options =
    rclcpp::NodeOptions()
    .start_parameter_services(false)
    .start_parameter_event_publisher(false)
    .arguments({"--ros-args", "-r", "__node:=" + generate_internal_node_name(prefix), "--"});
  return rclcpp::Node::make_shared("_", options);
}

LifecycleServiceClient::LifecycleServiceClient(const string & lifecycle_node_name)
: node_(generate_internal_node(lifecycle_node_name + "_lifecycle_client")),
  change_state_(lifecycle_node_name + "/change_state", node_),
  get_state_(lifecycle_node_name + "/get_state", node_)
{
}

LifecycleServiceClient::LifecycleServiceClient(
  const string & lifecycle_node_name,
  rclcpp::Node::SharedPtr parent_node)
: node_(parent_node),
  change_state_(lifecycle_node_name + "/change_state", node_),
  get_state_(lifecycle_node_name + "/get_state", node_)
{
}

void LifecycleServiceClient::change_state(
  const uint8_t transition,
  const seconds timeout)
{
  change_state_.wait_for_service(timeout);
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  change_state_.invoke(request, timeout);
}

bool LifecycleServiceClient::change_state(
  std::uint8_t transition)
{
  change_state_.wait_for_service();
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
  request->transition.id = transition;
  return change_state_.invoke(request, response);
}

uint8_t LifecycleServiceClient::get_state(
  const seconds timeout)
{
  get_state_.wait_for_service(timeout);
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}

}  // namespace ros2_utils

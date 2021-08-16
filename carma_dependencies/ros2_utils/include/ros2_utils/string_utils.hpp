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

#ifndef ROS2_UTILS__STRING_UTILS_HPP_
#define ROS2_UTILS__STRING_UTILS_HPP_

#include <string>
#include <vector>

namespace ros2_utils
{

typedef std::vector<std::string> Tokens;

// Remove a leading slash from a topic name
std::string strip_leading_slash(const std::string & in);

// Split a string at the delimiters
Tokens split(const std::string & tokenstring, char delimiter);

}  // namespace ros2_utils

#endif  // ROS2_UTILS__STRING_UTILS_HPP_

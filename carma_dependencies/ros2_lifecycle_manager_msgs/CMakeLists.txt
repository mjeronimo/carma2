cmake_minimum_required(VERSION 3.5)
project(ros2_lifecycle_manager_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ManageLifecycleNodes.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

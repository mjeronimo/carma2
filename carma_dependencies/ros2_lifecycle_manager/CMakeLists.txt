cmake_minimum_required(VERSION 3.5)
project(ros2_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ros2_lifecycle_manager_msgs REQUIRED)
find_package(ros2_utils REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)

#carma_package()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

set(dependencies
  geometry_msgs
  lifecycle_msgs
  ros2_lifecycle_manager_msgs
  ros2_utils
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
  bondcpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(lifecycle_manager
  src/main.cpp
)

target_link_libraries(lifecycle_manager
  ${library_name}
)

ament_target_dependencies(lifecycle_manager
  ${dependencies}
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  #add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()

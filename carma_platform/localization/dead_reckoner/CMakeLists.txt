cmake_minimum_required(VERSION 3.5)
project(dead_reckoner)

find_package(ament_cmake REQUIRED)
find_package(carma_common REQUIRED)
find_package(carma_utils REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_lifecycle_manager_msgs REQUIRED)
find_package(ros2_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

carma_package()

include_directories(
  include
)

set(executable_name ${PROJECT_NAME})
set(library_name ${executable_name}_core)

set(dependencies
  carma_common
  carma_utils
  message_filters
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_lifecycle_manager_msgs
  ros2_utils
  sensor_msgs
  tf2_ros
)

add_library(${library_name} SHARED
  src/dead_reckoner.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(distance_calculator SHARED plugins/distance_calculator.cpp)
ament_target_dependencies(distance_calculator ${dependencies})

add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} ${library_name} distance_calculator)
ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${library_name} distance_calculator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} distance_calculator)
ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME})

pluginlib_export_plugin_description_file(carma_utils plugins.xml)

ament_package()

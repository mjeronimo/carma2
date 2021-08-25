add_library(${library_name} SHARED
  cv_utils.cpp
  lifecycle_service_client.cpp
  lifecycle_utils.cpp
  node_thread.cpp
  node_utils.cpp
  param_utils.cpp
  string_utils.cpp
)

ament_target_dependencies(${library_name}
  cv_bridge
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  ros2_lifecycle_manager_msgs
  sensor_msgs
)

add_executable(lifecycle_bringup
  lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup ${library_name})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS lifecycle_bringup
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

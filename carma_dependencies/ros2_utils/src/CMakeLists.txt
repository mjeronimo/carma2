add_library(${library_name} SHARED
  lifecycle_service_client.cpp
  string_utils.cpp
  lifecycle_utils.cpp
  node_thread.cpp
)

ament_target_dependencies(${library_name}
  bondcpp
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  ros2_lifecycle_manager_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(lifecycle_bringup
  lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup ${library_name})

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_bringup
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

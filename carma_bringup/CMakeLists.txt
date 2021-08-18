cmake_minimum_required(VERSION 3.5)
project(carma_bringup)

find_package(ament_cmake REQUIRED)
find_package(carma_common REQUIRED)

carma_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

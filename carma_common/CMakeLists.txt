cmake_minimum_required(VERSION 3.5)

project(carma_common NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(carma_common)

ament_package(
  CONFIG_EXTRAS "carma_common-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

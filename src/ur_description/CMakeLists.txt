cmake_minimum_required(VERSION 3.8)
project(ur_description)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS bin/ur_arm_controller DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  lib/libmagnet_system.so
  DESTINATION lib)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(eyantra_kepler_colony)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  lib/libkepler_arena_plugin.so
  DESTINATION lib)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

install(
  DIRECTORY
    launch
    models
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

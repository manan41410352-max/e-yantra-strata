cmake_minimum_required(VERSION 3.8)
project(ebot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    meshes
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()

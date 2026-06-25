cmake_minimum_required(VERSION 3.8)
project(kitchen_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

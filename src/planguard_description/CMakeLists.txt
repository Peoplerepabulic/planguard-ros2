cmake_minimum_required(VERSION 3.10)
project(planguard_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes worlds launch config rviz
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()

cmake_minimum_required(VERSION 3.10)
project(planguard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ROS-free core: everything that decides a command lives here and is unit-tested.
add_library(planguard_core
  src/geometry.cpp
  src/cruise_law.cpp
  src/brake_filter.cpp
  src/qp2d.cpp
  src/cbf_filter.cpp)
target_include_directories(planguard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(cruise_controller src/cruise_controller_node.cpp)
target_link_libraries(cruise_controller planguard_core)
ament_target_dependencies(cruise_controller rclcpp geometry_msgs nav_msgs)

add_executable(safety_shield src/safety_shield_node.cpp)
target_link_libraries(safety_shield planguard_core)
ament_target_dependencies(safety_shield rclcpp geometry_msgs sensor_msgs std_msgs)

install(TARGETS planguard_core cruise_controller safety_shield
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_filters test/test_filters.cpp)
  target_link_libraries(test_filters planguard_core)
endif()

ament_package()

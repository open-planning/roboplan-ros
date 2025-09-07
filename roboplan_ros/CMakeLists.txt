cmake_minimum_required(VERSION 3.22)
project(roboplan_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(roboplan REQUIRED)

if(BUILD_TESTING)
endif()

ament_package()

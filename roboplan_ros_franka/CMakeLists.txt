cmake_minimum_required(VERSION 3.22)
project(roboplan_ros_franka)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(roboplan REQUIRED)
find_package(roboplan_example_models REQUIRED)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/
        DESTINATION lib/${PROJECT_NAME}
        USE_SOURCE_PERMISSIONS
        FILES_MATCHING PATTERN "*.py")

ament_package()

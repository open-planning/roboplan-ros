cmake_minimum_required(VERSION 3.22)
project(roboplan_ros_franka)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(roboplan REQUIRED)
find_package(roboplan_example_models REQUIRED)

install(DIRECTORY config description launch
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/optimal_ik_streaming_node.py
        scripts/plan_and_execute_node.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()

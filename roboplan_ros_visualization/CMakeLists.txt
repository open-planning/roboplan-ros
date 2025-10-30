cmake_minimum_required(VERSION 3.22)
project(roboplan_ros_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboplan REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(roboplan_ros_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Library package
add_library(roboplan_ros_visualization SHARED
  src/interactive_marker_ik.cpp
)
target_include_directories(roboplan_ros_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_ros_visualization PUBLIC
  rclcpp::rclcpp
  roboplan::roboplan
  roboplan_simple_ik::roboplan_simple_ik
  roboplan_ros_cpp::roboplan_ros_cpp
  ${interactive_markers_TARGETS}
  ${visualization_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
set_property(TARGET roboplan_ros_visualization PROPERTY CXX_STANDARD 20)

# Install all libs and executables
install(
  TARGETS roboplan_ros_visualization
  EXPORT roboplan_ros_visualization-targets
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_python_install_package(${PROJECT_NAME})

ament_export_targets(roboplan_ros_visualization-targets HAS_LIBRARY_TARGET)
ament_package()

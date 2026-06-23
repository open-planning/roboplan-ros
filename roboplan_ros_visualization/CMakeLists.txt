cmake_minimum_required(VERSION 3.22)
project(roboplan_ros_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(roboplan REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(roboplan_ros_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tl_expected REQUIRED)

add_library(roboplan_ros_visualization SHARED
    src/roboplan_visualizer.cpp
    src/roboplan_ik_marker.cpp
    src/path_visualization.cpp
)
target_include_directories(roboplan_ros_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_ros_visualization
  PUBLIC
    roboplan::roboplan
    roboplan_simple_ik::roboplan_simple_ik
    roboplan_ros_cpp::roboplan_ros_cpp
    pinocchio::pinocchio
    ${tf2_eigen_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${builtin_interfaces_TARGETS}
    ${interactive_markers_TARGETS}
)
set_property(TARGET roboplan_ros_visualization PROPERTY CXX_STANDARD 20)
ament_export_dependencies(
  roboplan
  Eigen3
  pinocchio
  tf2_eigen
  trajectory_msgs
  geometry_msgs
)
install(
  TARGETS
  roboplan_ros_visualization
  EXPORT roboplan_ros_visualization-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

# Optionally build the roboplan_ros_visualization bindings.
# Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_ros_visualization Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_package()

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

# For nanobind python bindings
find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
find_package(roboplan_bindings REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

# Detect the installed nanobind package and import it into CMake
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

# Compile the nanobind python bindings as a subpackage of the python package.
nanobind_add_module(
  roboplan_ros_visualization_bindings
  STABLE_ABI
  NB_STATIC
  NB_SUPPRESS_WARNINGS
  src/roboplan_ros_visualization_bindings.cpp
)
target_include_directories(roboplan_ros_visualization_bindings PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  ${rosidl_generator_cpp_INCLUDE_DIRS}
)
set_property(TARGET roboplan_ros_visualization_bindings PROPERTY CXX_STANDARD 20)
target_link_libraries(roboplan_ros_visualization_bindings PRIVATE
  roboplan_ros_visualization
  roboplan_ros_cpp::roboplan_ros_cpp
  roboplan::roboplan
  roboplan_bindings::roboplan_bindings_utils
  ${rosidl_generator_cpp_LIBRARIES}
  tl_expected::tl_expected
)

# This package has a python module as well, so this install that first and then
# install the bindings a sub-package.
ament_python_install_package(${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
set_target_properties(roboplan_ros_visualization_bindings PROPERTIES
  OUTPUT_NAME "bindings"
)
install(TARGETS roboplan_ros_visualization_bindings
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/roboplan_ros_visualization"
)

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_package()

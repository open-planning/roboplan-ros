cmake_minimum_required(VERSION 3.22)
project(roboplan_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(roboplan REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(roboplan_ros SHARED
    include/roboplan_ros/type_conversions.hpp
)
target_include_directories(roboplan_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_ros
    roboplan::roboplan
    trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp
)
set_property(TARGET roboplan_ros PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  roboplan_ros
  EXPORT roboplan_ros-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

# Try building python bindings by fetching nanobind directly...
find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
include(FetchContent)
FetchContent_Declare(
  nanobind
  GIT_REPOSITORY https://github.com/wjakob/nanobind.git
  GIT_TAG v2.9.2
)
FetchContent_MakeAvailable(nanobind)

nanobind_add_module(
  roboplan_ros_py
  NB_SHARED
  python/bindings.cpp
)

# Giving a different name prevents clobbering on imports when running from the workspace root
set_target_properties(roboplan_ros_py PROPERTIES
  OUTPUT_NAME roboplan_ros_py
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python
)

target_link_libraries(roboplan_ros_py PRIVATE
    roboplan_ros
    roboplan::roboplan
    trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp
    builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp
)
set_property(TARGET roboplan_ros_py PROPERTY CXX_STANDARD 20)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS roboplan_ros_py
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}"
)

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_export_targets(roboplan_ros-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(roboplan trajectory_msgs builtin_interfaces)

ament_package()

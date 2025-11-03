cmake_minimum_required(VERSION 3.22)
project(roboplan_ros_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(roboplan REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(roboplan_ros_cpp SHARED
    src/type_conversions.cpp
    src/kinematics.cpp
)
target_include_directories(roboplan_ros_cpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_ros_cpp
  PUBLIC
    roboplan::roboplan
    roboplan_simple_ik::roboplan_simple_ik
    Eigen3::Eigen
    pinocchio::pinocchio
    ${tf2_eigen_TARGETS}
    ${trajectory_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
)
set_property(TARGET roboplan_ros_cpp PROPERTY CXX_STANDARD 20)
ament_export_dependencies(
  roboplan
  roboplan_simple_ik
  Eigen3
  pinocchio
  tf2_eigen
  trajectory_msgs
  geometry_msgs
)
install(
  TARGETS
  roboplan_ros_cpp
  EXPORT roboplan_ros_cpp-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
ament_export_targets(roboplan_ros_cpp-targets HAS_LIBRARY_TARGET)
ament_package()

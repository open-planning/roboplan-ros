cmake_minimum_required(VERSION 3.22)
project(roboplan_ros_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs)
find_package(pinocchio REQUIRED)
find_package(roboplan REQUIRED)
find_package(trajectory_msgs)

add_library(roboplan_ros_cpp SHARED
    src/type_conversions.cpp
)
target_include_directories(roboplan_ros_cpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_ros_cpp
    roboplan::roboplan
    Eigen3::Eigen
    pinocchio::pinocchio
    ${trajectory_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
)
set_property(TARGET roboplan_ros_cpp PROPERTY CXX_STANDARD 20)

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

ament_package()

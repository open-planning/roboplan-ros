cmake_minimum_required(VERSION 3.22)
project(roboplan_roscpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(roboplan REQUIRED)
find_package(trajectory_msgs)

add_library(roboplan_roscpp SHARED
    include/roboplan_roscpp/type_conversions.hpp
)
target_include_directories(roboplan_roscpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_roscpp
    roboplan::roboplan
)
ament_target_dependencies(roboplan_roscpp trajectory_msgs)
set_property(TARGET roboplan_roscpp PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  roboplan_roscpp
  EXPORT roboplan_roscpp-targets
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

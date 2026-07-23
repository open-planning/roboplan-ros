cmake_minimum_required(VERSION 3.22)
project(roboplan_ros_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
  # clang-cl remaps plain -Wall to MSVC /Wall (clang's -Weverything); use the
  # /clang: prefix to pass the GNU-style flags through unchanged.
  add_compile_options(/clang:-Wall /clang:-Wextra /clang:-Wpedantic)
elseif(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Windows: auto-export all symbols so DLLs get import libraries.
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# macOS: strip unused dylibs (like GNU --as-needed on Linux) so the Python
# generator libraries exported by the ROS interface targets stay out of the
# dylib closure; they reference Python symbols plain C++ binaries cannot load.
if(APPLE)
  add_link_options(LINKER:-dead_strip_dylibs)
endif()

find_package(ament_cmake REQUIRED)
find_package(roboplan REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(roboplan_ros_cpp SHARED
    src/type_conversions.cpp
)
target_include_directories(roboplan_ros_cpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_ros_cpp
  PUBLIC
    roboplan::roboplan
    Eigen3::Eigen
    pinocchio::pinocchio
    ${tf2_eigen_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${trajectory_msgs_TARGETS}
    ${builtin_interfaces_TARGETS}
)
set_property(TARGET roboplan_ros_cpp PROPERTY CXX_STANDARD 20)
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
  roboplan_ros_cpp
  EXPORT roboplan_ros_cpp-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

# Optionally build the roboplan_ros_cpp bindings.
# Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_ros_cpp Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_export_targets(roboplan_ros_cpp-targets HAS_LIBRARY_TARGET)
ament_package()

cmake_minimum_required(VERSION 3.22)
project(roboplan_ros_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(roboplan REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tl_expected REQUIRED)
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

# Compile the nanobind python bindings for type conversions.
nanobind_add_module(
  bindings
  STABLE_ABI
  NB_STATIC
  src/bindings.cpp
)
target_include_directories(bindings PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  ${rosidl_generator_cpp_INCLUDE_DIRS}
)
set_property(TARGET bindings PROPERTY CXX_STANDARD 20)
target_link_libraries(bindings PRIVATE
  roboplan_ros_cpp
  roboplan::roboplan
  roboplan_bindings::roboplan_bindings_utils
  ${rosidl_generator_cpp_LIBRARIES}
  tl_expected::tl_expected
)
ament_python_install_package(${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS bindings
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/roboplan_ros_cpp"
)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
  DESTINATION "${PYTHON_INSTALL_DIR}/roboplan_ros_cpp"
)

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
ament_export_targets(roboplan_ros_cpp-targets HAS_LIBRARY_TARGET)
ament_package()

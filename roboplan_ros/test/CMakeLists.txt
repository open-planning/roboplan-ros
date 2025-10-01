find_package(GTest REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_executable(test_type_conversions test_type_conversions.cpp)
set_property(TARGET test_type_conversions PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_type_conversions
  roboplan_ros
  GTest::gtest_main
)
ament_add_gtest_test(test_type_conversions)

find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(test_bindings test_bindings.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

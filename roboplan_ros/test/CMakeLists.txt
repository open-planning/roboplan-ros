find_package(GTest REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_executable(test_types test_types.cpp)
set_property(TARGET test_types PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_types
  roboplan_ros
  GTest::gtest_main
)
ament_add_gtest_test(test_types)

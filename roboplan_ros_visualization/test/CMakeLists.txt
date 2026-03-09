find_package(GTest REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

add_executable(roboplan_visualizer_test_cpp roboplan_visualizer_test.cpp)
set_property(TARGET roboplan_visualizer_test_cpp PROPERTY CXX_STANDARD 20)
target_link_libraries(
  roboplan_visualizer_test_cpp
  roboplan_ros_visualization
  GTest::gtest_main
  ament_index_cpp::ament_index_cpp
)
ament_add_gtest_test(test_type_conversions_cpp)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
ament_add_pytest_test(roboplan_visualizer_test_py roboplan_visualizer_test.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
)

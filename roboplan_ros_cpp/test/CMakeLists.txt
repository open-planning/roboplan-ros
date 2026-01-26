find_package(GTest REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(roboplan_example_models REQUIRED)

install(DIRECTORY resources
        DESTINATION share/${PROJECT_NAME}/test)

add_executable(test_type_conversions_cpp test_type_conversions.cpp)
set_property(TARGET test_type_conversions_cpp PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_type_conversions_cpp
  roboplan_ros_cpp
  GTest::gtest_main
  ament_index_cpp::ament_index_cpp
)
ament_add_gtest_test(test_type_conversions_cpp)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
ament_add_pytest_test(test_type_conversions_py test_type_conversions.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
)

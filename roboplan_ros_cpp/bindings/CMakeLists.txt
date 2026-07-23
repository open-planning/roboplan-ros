# Compile the nanobind python bindings for roboplan_ros_cpp.

# For nanobind python bindings
find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

# Rely on tl_expected if found, else look at the roboplan alias.
find_package(tl_expected QUIET)
if (NOT tl_expected_FOUND)
  add_library(tl_expected::tl_expected ALIAS roboplan::tl_expected)
endif()

# Detect the installed nanobind package and import it into CMake
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(
  _cpp_ext
  STABLE_ABI
  NB_STATIC
  src/roboplan_ros_cpp_bindings.cpp
)
target_include_directories(_cpp_ext PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  ${rosidl_generator_cpp_INCLUDE_DIRS}
)
set_property(TARGET _cpp_ext PROPERTY CXX_STANDARD 20)
target_link_libraries(_cpp_ext PRIVATE
  roboplan_ros_cpp
  roboplan::roboplan
  ${rosidl_generator_cpp_LIBRARIES}
  tl_expected::tl_expected
)

# Manually install rather than rely on ament python packaging to match core roboplan's
# standard for ROS bindings. This will put bindings into `roboplan_ros.<package>`.
ament_get_python_install_dir(_ament_py_dir)
if(_ament_py_dir MATCHES "dist-packages")
  set(PYTHON_DESTINATION
      "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages")
else()
  set(PYTHON_DESTINATION "${_ament_py_dir}")
endif()
install(TARGETS _cpp_ext LIBRARY DESTINATION "${PYTHON_DESTINATION}/roboplan_ros/cpp")

# Generate Python stub files from the bindings.
# Older nanobind versions (which can come from rosdep on certain distros) do not provide
# nanobind_add_stub, so skip generation rather than failing the configure step.
# See https://github.com/ros/rosdistro/pull/49894 for more details.
# Default OFF on Windows: stub generation imports the extension from the build
# tree, where the DLLs it depends on are not yet on the loader search path.
# The committed stubs are generated on Unix.
if(WIN32)
  set(generate_stubs_default OFF)
else()
  set(generate_stubs_default ON)
endif()
option(GENERATE_PYTHON_STUBS "Generate the Python stubs associated with the bindings" ${generate_stubs_default})
if(GENERATE_PYTHON_STUBS AND NOT COMMAND nanobind_add_stub)
  message(WARNING
    "Skipping Python stub generation: the installed nanobind version does not "
    "provide nanobind_add_stub. Upgrade nanobind to generate stubs.")
endif()

if(GENERATE_PYTHON_STUBS AND COMMAND nanobind_add_stub)
  nanobind_add_stub(
    _cpp_ext_stub
    MODULE _cpp_ext
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan_ros/cpp/_cpp_ext.pyi"
    MARKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan_ros/cpp/py.typed"
    PYTHON_PATH "$<TARGET_FILE_DIR:_cpp_ext>"
    DEPENDS _cpp_ext
  )
endif()

install(DIRECTORY python/roboplan_ros/ DESTINATION "${PYTHON_DESTINATION}/roboplan_ros")
install(DIRECTORY include/ DESTINATION include)

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(roboplan_example_models REQUIRED)
  ament_add_pytest_test(test_type_conversions_py test/test_type_conversions.py)
endif()

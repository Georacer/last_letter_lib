cmake_minimum_required(VERSION 3.15)
project(last_letter_lib VERSION 2.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json

message(STATUS "Building ${PROJECT_NAME}")

# Set compiler standards
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Whole project builds at C++17 (required by the DataTamer logging backend).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependences
# find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# include(FindEigen3.cmake)
find_package(LAPACK REQUIRED)

find_package(Boost REQUIRED)

# find_package(NLopt REQUIRED) # Will enable later with trimmer
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Current Python site-packages dir: ${Python3_SITELIB}")
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 PATHS ${Python3_SITELIB}/pybind11/share/cmake/pybind11 CONFIG REQUIRED)

# add_subdirectory(external/pybind11)

# Path where libyaml-cpp.so will be built
set(YAML_CPP_SO_PATH ${CMAKE_BINARY_DIR}/ext-yaml-cpp-prefix/src/ext-yaml-cpp-build/libyaml-cpp.so)

# Import yaml-cpp
include(FetchContent)

FetchContent_Declare(
  yaml_cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG        yaml-cpp-0.9.0
)

# Set options before the library is added
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(yaml_cpp)
# Ensure the yaml-cpp library (and other files) will get installed before they get cleaned.
install(TARGETS yaml-cpp
        EXPORT yamlcppTargets
        LIBRARY DESTINATION ${PROJECT_NAME}
        ARCHIVE DESTINATION ${PROJECT_NAME}
        RUNTIME DESTINATION ${PROJECT_NAME}
)

# Import DataTamer, the data-logging backend (MCAP output, PlotJuggler-native).
# The buildable CMake project lives in the data_tamer_cpp/ subdirectory, hence
# SOURCE_SUBDIR (honored by FetchContent_MakeAvailable on CMake >= 3.18).
# DATA_TAMER_BUILD_ROS must be OFF so it uses its vendored mcap (3rdparty/mcap)
# instead of requiring a ROS 2 / mcap_vendor environment.
FetchContent_Declare(
  data_tamer
  GIT_REPOSITORY https://github.com/PickNikRobotics/data_tamer.git
  GIT_TAG        1.0.3
  SOURCE_SUBDIR  data_tamer_cpp
)
set(DATA_TAMER_BUILD_ROS OFF CACHE BOOL "" FORCE)
set(DATA_TAMER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(DATA_TAMER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
# Build DataTamer and its vendored mcap as STATIC, position-independent, so they
# embed directly into last_letter_lib.so. Otherwise BUILD_SHARED_LIBS (forced ON
# earlier for yaml-cpp) makes them separate .so files that are not shipped next
# to the Python extension, so `import last_letter_lib` fails with
# "libdata_tamer.so: cannot open shared object file". Scope the override to this
# fetch and restore BUILD_SHARED_LIBS afterwards so yaml-cpp/googletest are
# unaffected.
set(_LLL_BUILD_SHARED_LIBS_SAVED "${BUILD_SHARED_LIBS}")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(data_tamer)
set(BUILD_SHARED_LIBS "${_LLL_BUILD_SHARED_LIBS_SAVED}" CACHE BOOL "" FORCE)
# DataTamer compiles its own target with -Werror; drop it so a compiler version
# other than the one upstream tested can't fail our build on a stray warning.
if(TARGET data_tamer AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  target_compile_options(data_tamer PRIVATE -Wno-error)
endif()


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}

  # "/usr/local/include" # Used to include nlopt
  # ${YAML_CPP_INCLUDE_DIR}
)

# Declare the project library
add_library(last_letter_lib SHARED
  src/math_utils.cpp
  src/uav_utils.cpp
  src/geo_mag_declination.cpp
  src/prog_utils.cpp
  src/systems.cpp
  src/environment.cpp
  src/gravity.cpp
  src/aerodynamics.cpp
  src/propulsion/propulsion.cpp
  src/propulsion/no_engine.cpp
  src/propulsion/thruster_simple.cpp
  src/propulsion/beard_engine.cpp
  src/propulsion/piston_engine.cpp
  src/propulsion/electric_engine.cpp
  src/propulsion/omega_control_engine.cpp
  src/ground_reaction/ground_reaction.cpp
  src/ground_reaction/no_ground_reactions.cpp
  src/ground_reaction/panos_contact_points.cpp
  src/ground_reaction/point_friction.cpp
  src/dynamics.cpp
  src/kinematics.cpp
  src/sensors.cpp
  src/uav_model.cpp
  src/logging/logging.cpp
)
target_link_libraries(last_letter_lib
  Eigen3::Eigen
  ${LAPACK_LIBRARIES}
  yaml-cpp
  Boost::boost
  data_tamer
)

# add_library(lib_trimmer src/trimmer.cpp)
# target_link_libraries(lib_trimmer
# ${NLOPT_LIBRARIES}
# lib_uav_model)
list(APPEND pybind11_sources
  src/python_api.cpp
)
pybind11_add_module(cpp_last_letter_lib ${pybind11_sources})
set_target_properties(cpp_last_letter_lib PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_RPATH "$ORIGIN"
)
target_link_libraries(cpp_last_letter_lib PRIVATE last_letter_lib)
install(TARGETS cpp_last_letter_lib LIBRARY DESTINATION ${PROJECT_NAME})

# Copy data files to home folder
# file(COPY last_letter_models DESTINATION $ENV{HOME}) # Deprecated, using new models folder
file(COPY models DESTINATION $ENV{HOME}/last_letter_models)
file(COPY last_letter_models/HID.yaml DESTINATION $ENV{HOME}/last_letter_models)

# Export the header files for use by consumers of the library
# ament_export_include_directories(include)  # Unnecessary according to https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/#building-a-library
# Export the library binaries themselves
# ament_export_libraries(
# last_letter_lib
# )  # Unnecessary according to https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/#building-a-library

# # Export the libraries
# ament_export_targets(
# export_last_letterlib HAS_LIBRARY_TARGET
# )
# # Declare dependencies of this library to its consumers
# ament_export_dependencies(
# Eigen3
# LAPACK
# )

# Compile test files
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
)

FetchContent_MakeAvailable(googletest)

enable_testing()

list(APPEND test_sources
  tests/cpp/test_utils.cpp
  tests/cpp/test_prog_utils.cpp
  tests/cpp/test_uav_utils.cpp
  tests/cpp/test_math_utils.cpp
  tests/cpp/test_systems.cpp
  tests/cpp/test_environment.cpp
  tests/cpp/test_aerodynamics.cpp
  tests/cpp/test_propulsion.cpp
  tests/cpp/test_ground_reaction.cpp
  tests/cpp/test_gravity.cpp
  tests/cpp/test_dynamics.cpp
  tests/cpp/test_kinematics.cpp
  tests/cpp/test_uav_model.cpp
  tests/cpp/test_loop_rate.cpp

  # tests/cpp/test_pybind.cpp
)
add_executable(all_tests ${test_sources})
target_include_directories(all_tests PUBLIC
  tests/cpp
)
target_link_libraries(all_tests
  GTest::gtest_main
  yaml-cpp
  last_letter_lib
)

# Perhaps copy over the test data to the build folder?
include(GoogleTest)
gtest_discover_tests(all_tests)

# # the following line skips the linter which checks for copyrights
# # uncomment the line when a copyright and license is not present in all source files
# #set(ament_cmake_copyright_FOUND TRUE)
# # the following line skips cpplint (only works in a git repo)
# # uncomment the line when this package is not in a git repo
# #set(ament_cmake_cpplint_FOUND TRUE)

# install(DIRECTORY tests/cpp/test_parameters DESTINATION share/${PROJECT_NAME})

# endif()
install(
  DIRECTORY include/ DESTINATION include
)

list(APPEND exported_libs
  last_letter_lib
)
install(
  TARGETS ${exported_libs}
  LIBRARY DESTINATION ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  EXPORT last_letter_lib_targets
)


# install(
# TARGETS last_letter_lib
# EXPORT export_last_letter_lib
# LIBRARY DESTINATION lib
# ARCHIVE DESTINATION lib
# RUNTIME DESTINATION bin
# INCLUDES DESTINATION include
# )

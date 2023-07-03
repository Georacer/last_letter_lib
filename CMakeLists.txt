cmake_minimum_required(VERSION 3.15)
project(last_letter_lib VERSION 2.1.0)

# Set compiler standards
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Configure compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependences
# find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# include(FindEigen3.cmake)
find_package(LAPACK REQUIRED)

# find_package(NLopt REQUIRED) # Will enable later with trimmer
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Current Python site-packages dir: ${Python3_SITELIB}")
find_package(pybind11 PATHS ${Python3_SITELIB}/pybind11/share/cmake/pybind11 CONFIG REQUIRED)

# add_subdirectory(external/pybind11)

# Declare and compile the yaml-cpp project
include(ExternalProject)
ExternalProject_Add(ext-yaml-cpp
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/external/yaml-cpp
  CMAKE_ARGS "-DYAML_CPP_BUILD_TESTS=OFF"
  "-DYAML_CPP_BUILD_TOOLS=OFF"
  "-DBUILD_SHARED_LIBS=ON"
  INSTALL_COMMAND "" # Do not install in system, library needed only locally
  USES_TERMINAL_BUILD true
)

# Construct include directories path
ExternalProject_Get_Property(ext-yaml-cpp SOURCE_DIR) # Access the source directory
set(YAML_CPP_INCLUDE_DIR ${SOURCE_DIR}/include) # Aggregate headers to variable
message(STATUS "YAML_CPP_INCLUDE_DIR set to ${YAML_CPP_INCLUDE_DIR}")

# Expose built library
add_library(yaml-cpp SHARED IMPORTED) # Declare the shared library
ExternalProject_Get_Property(ext-yaml-cpp BINARY_DIR) # Access the binaries location
set_target_properties(yaml-cpp PROPERTIES
  IMPORTED_LOCATION ${BINARY_DIR}/libyaml-cpp.so # Attach the .so file to the library
  INTERFACE_INCLUDE_DIRECTORIES ${YAML_CPP_INCLUDE_DIR}
)
add_dependencies(yaml-cpp ext-yaml-cpp) # Declare a dependency on the 3rd party project, just in case
set(YAML_CPP_LIBRARIES yaml-cpp) # Aggregate all libraries to variable
message(STATUS "yaml-cpp library (YAML_CPP_LIBRARIES) found at ${YAML_CPP_LIBRARIES}")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}

  # "/usr/local/include" # Used to include nlopt
  ${YAML_CPP_INCLUDE_DIR}
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
  src/ground_reaction/ground_reaction.cpp
  src/link.cpp
  src/dynamics.cpp
  src/kinematics.cpp
  src/sensors.cpp
  src/uav_model.cpp
)
target_link_libraries(last_letter_lib
  Eigen3::Eigen
  ${LAPACK_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# add_library(lib_trimmer src/trimmer.cpp)
# target_link_libraries(lib_trimmer
# ${NLOPT_LIBRARIES}
# lib_uav_model)
list(APPEND pybind11_sources
  src/python_api.cpp
)
pybind11_add_module(cpp_last_letter_lib ${pybind11_sources})
target_link_libraries(cpp_last_letter_lib PRIVATE last_letter_lib)

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
  tests/cpp/test_gravity.cpp
  tests/cpp/test_dynamics.cpp
  tests/cpp/test_uav_model.cpp

  # tests/cpp/test_pybind.cpp
)
add_executable(all_tests ${test_sources})
target_include_directories(all_tests PUBLIC
  tests/cpp
)
target_link_libraries(all_tests
  GTest::gtest_main
  ${YAML_CPP_LIBRARIES}
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
  LIBRARY DESTINATION lib
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

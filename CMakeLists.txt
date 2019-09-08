cmake_minimum_required(VERSION 2.8.3)
project(uav_model)

find_package(Eigen3 REQUIRED)

find_package(LAPACK REQUIRED)
# Have yaml-cpp already copied and built
find_package(yaml-cpp PATHS external/yaml-cpp/build)

include_directories(
  include
  include/propulsion
  include/ground_reaction
  ${EIGEN3_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
)

add_library(math_utils src/math_utils.cpp)
target_link_libraries(math_utils ${LAPACK_LIBRARIES} Eigen3::Eigen)

add_library(uav_utils src/uav_utils.cpp)
target_link_libraries(uav_utils Eigen3::Eigen)

add_library(prog_utils src/prog_utils.cpp)
target_link_libraries(prog_utils math_utils yaml-cpp)

add_library(lib_environment src/environment.cpp)
target_link_libraries(lib_environment math_utils uav_utils Eigen3::Eigen)

add_library(lib_gravity src/gravity.cpp)
target_link_libraries(lib_gravity math_utils uav_utils Eigen3::Eigen)

add_library(lib_aerodynamics src/aerodynamics.cpp)
target_link_libraries(lib_aerodynamics prog_utils uav_utils math_utils lib_environment yaml-cpp Eigen3::Eigen)

add_library(lib_propulsion src/propulsion/propulsion.cpp)
target_link_libraries(lib_propulsion uav_utils math_utils prog_utils lib_environment yaml-cpp Eigen3::Eigen)

add_library(lib_ground_reaction src/ground_reaction/ground_reaction.cpp)
target_link_libraries(lib_ground_reaction uav_utils math_utils prog_utils yaml-cpp Eigen3::Eigen)

add_library(lib_uav_model src/dynamics.cpp src/kinematics.cpp src/uav_model.cpp)
target_link_libraries(lib_uav_model uav_utils 
                                    math_utils
                                    yaml-cpp
                                    prog_utils
                                    lib_environment
                                    lib_gravity
                                    lib_propulsion
                                    lib_aerodynamics
                                    lib_ground_reaction
                                    Eigen3::Eigen)


# Compile test files
add_executable(test_yaml test/test_yaml.cpp)
target_link_libraries(test_yaml prog_utils yaml-cpp)

add_executable(test_environment test/test_environment.cpp)
target_link_libraries(test_environment lib_environment prog_utils yaml-cpp)

add_executable(test_gravity test/test_gravity.cpp)
target_link_libraries(test_gravity lib_gravity)

add_executable(test_aerodynamics test/test_aerodynamics.cpp)
target_link_libraries(test_aerodynamics yaml-cpp lib_aerodynamics)

add_executable(test_propulsion test/test_propulsion.cpp)
target_link_libraries(test_propulsion lib_propulsion)

add_executable(test_ground_reaction test/test_ground_reaction.cpp)
target_link_libraries(test_ground_reaction lib_ground_reaction)

add_executable(test_dynamics src/dynamics.cpp test/test_dynamics.cpp )
target_link_libraries(test_dynamics uav_utils
                                    math_utils
                                    prog_utils
                                    lib_environment
                                    lib_gravity
                                    lib_propulsion
                                    lib_aerodynamics
                                    lib_ground_reaction
                                    yaml-cpp
                                    Eigen3::Eigen)

add_executable(test_kinematics src/kinematics.cpp test/test_kinematics.cpp )
target_link_libraries(test_kinematics uav_utils
                                      math_utils
                                      prog_utils
                                      lib_environment
                                      lib_gravity
                                      lib_propulsion
                                      lib_aerodynamics
                                      lib_ground_reaction
                                      yaml-cpp
                                      Eigen3::Eigen)

add_executable(test_uav_model test/test_uav_model.cpp)
target_link_libraries(test_uav_model lib_uav_model)
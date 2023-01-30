# ament_add_gtest(test_math_utils test_match_utils.cpp)
# ament_target_dependencies(test_math_utils math_utils)

ament_add_gtest(test_uav_utils test_uav_utils.cpp)
target_link_libraries(test_uav_utils uav_utils)

# ament_add_gtest_executable(test_yaml test/test_yaml.cpp)
# target_link_libraries(test_yaml prog_utils yaml-cpp)

# ament_add_gtest_executable(test_prog_utils test/test_prog_utils.cpp)
# target_link_libraries(test_prog_utils prog_utils uav_utils)

# ament_add_gtest_executable(test_environment test/test_environment.cpp)
# target_link_libraries(test_environment lib_environment prog_utils yaml-cpp)

# ament_add_gtest_executable(test_gravity test/test_gravity.cpp)
# target_link_libraries(test_gravity lib_gravity)

# ament_add_gtest_executable(test_aerodynamics test/test_aerodynamics.cpp)
# target_link_libraries(test_aerodynamics yaml-cpp lib_aerodynamics)

# ament_add_gtest_executable(test_propulsion test/test_propulsion.cpp)
# target_link_libraries(test_propulsion lib_propulsion)

# ament_add_gtest_executable(test_rotations test/test_rotations.cpp)
# target_link_libraries(test_rotations lib_gravity Eigen3::Eigen)

# ament_add_gtest_executable(test_ground_reaction test/test_ground_reaction.cpp)
# target_link_libraries(test_ground_reaction lib_ground_reaction)

# ament_add_gtest_executable(test_dynamics src/dynamics.cpp test/test_dynamics.cpp )
# target_link_libraries(test_dynamics uav_utils
#                           math_utils
#                           prog_utils
#                           lib_environment
#                           lib_gravity
#                           lib_propulsion
#                           lib_aerodynamics
#                           lib_ground_reaction
#                           yaml-cpp
#                           Eigen3::Eigen
#                          )

# ament_add_gtest_executable(test_kinematics src/kinematics.cpp test/test_kinematics.cpp )
# target_link_libraries(test_kinematics uav_utils
#                           math_utils
#                           prog_utils
#                           lib_environment
#                           lib_gravity
#                           lib_propulsion
#                           lib_aerodynamics
#                           lib_ground_reaction
#                           yaml-cpp
#                           Eigen3::Eigen
#                          )

# ament_add_gtest_executable(test_uav_model test/test_uav_model.cpp)
# target_link_libraries(test_uav_model lib_uav_model)

# ament_add_gtest_executable(test_parameters test/test_parameters.cpp)
# target_link_libraries(test_parameters lib_uav_model)

# ament_add_gtest_executable(test_loop_rate test/test_loop_rate.cpp)
# target_link_libraries(test_loop_rate lib_uav_model)

# ament_add_gtest_executable(test_trimmer_input test/test_trimmer_input.cpp)
# target_link_libraries(test_trimmer_input lib_trimmer)

# ament_add_gtest_executable(test_trimmer_state test/test_trimmer_state.cpp)
# target_link_libraries(test_trimmer_state lib_trimmer)

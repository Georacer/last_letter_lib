#!/bin/bash

set -e

test_binary_location="../build/"
uav_name="skywalker_2013"

test_list[0]="test_rotations"
test_list[1]="test_yaml"
test_list[2]="test_prog_utils ${uav_name}"
test_list[3]="test_environment"
test_list[4]="test_gravity"
test_list[5]="test_aerodynamics ${uav_name}"
test_list[6]="test_propulsion ${uav_name}"
test_list[7]="test_ground_reaction ${uav_name}"
test_list[8]="test_dynamics ${uav_name}"
test_list[9]="test_kinematics ${uav_name}"
test_list[10]="test_uav_model ${uav_name}"
test_list[11]="test_loop_rate ${uav_name}"
test_list[12]="test_trimmer ${uav_name}"

for test_file in "${test_list[@]}"
do
    echo ">>> Running test file '${test_file}'"
    $test_binary_location$test_file
    echo ""
done
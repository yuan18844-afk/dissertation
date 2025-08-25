#!/bin/bash


timestamp=$(date +%Y%m%d_%H%M%S)


label=$1   #  ./record_all.sh static1

python3 $(rospack find franka_digital_twin)/scripts/record_gazebo_joint_states.py \
    --topic /joint_states --output ~/gazebo_${label}_${timestamp}.csv &




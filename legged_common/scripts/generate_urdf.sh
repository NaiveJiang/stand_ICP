#!/usr/bin/env sh
mkdir -p /home/lpy/waao_hunter/hunter_control
rosrun xacro xacro $1 robot_type:=$2 > /tmp/hunter_control/$2.urdf
# rosrun xacro xacro /home/lpy/waao_hunter/src/hunter_bipedal_control/legged_examples/legged_hunter/legged_hunter_description/urdf/robot.xacro > /home/lpy/waao_hunter/hunter_control/hunter.urdf

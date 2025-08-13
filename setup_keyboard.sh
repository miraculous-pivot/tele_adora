#!/bin/bash

echo "这是key board控制方式的控制端"
cd ./leader/key_board
colcon build 
source install/setup.bash
ros2 run my_teleop_pkg bimanual_teleop 
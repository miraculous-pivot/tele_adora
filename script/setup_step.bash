#!/bin/bash
cd ..
pwd
echo "这是足式控制器控制方式的控制端"
cd ./leader/dual_encoder_communication 
colcon build --packages-select brt_encoder_msgs
source install/setup.bash
colcon build --packages-select brt_encoder_reader
source install/setup.bash 
ros2 launch brt_encoder_reader encoder.launch.py

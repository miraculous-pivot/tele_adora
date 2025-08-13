#!/bin/bash

echo "这是VR控制方式的服务端"

#todo:
#检查环境
#检查依赖
#安装依赖
#构建一些源码安装的包
#id号绑定

# 启动Orbbec摄像头（独立终端）
cd ./slave/video/camera/orbbecsdk
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
echo "=== 启动Orbbec摄像头 ==="
gnome-terminal --title="ORBBEC CAMERA" --working-directory="$PWD" -- \
  bash -c ". ./install/setup.bash;
           ros2 launch orbbec_camera astra.launch.py;
           exec bash"

cd ../../../../


# 在adora目录下统一构建底盘和升降模块
echo "=== 在 adora 目录下统一构建底盘和升降模块 ==="
cd ./slave/adora
colcon build
source install/setup.bash

# 启动提升电机控制模块（独立终端）
gnome-terminal --title="LIFTING MOTOR CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动 lifting_motor_control 节点 ==='; 
           sudo chmod 777 /dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00;
           ros2 launch adora_lifting_motor_control adora_a2_max_ros2_node.py;
           exec bash"

# 等待2秒确保终端启动完成
sleep 2

# 启动底盘控制模块（独立终端）
gnome-terminal --title="CHASSIS CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动 chassis_control 节点 ==='; 
           sudo chmod 777 /dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00;
           ros2 launch adora_chassis_bringup adora_a2_max_ros2.launch.py;
           exec bash"

# 等待2秒确保终端启动完成
sleep 2

# 启动云台控制模块（独立终端）
gnome-terminal --title="GIMBAL CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动 gimbal_control 节点 ==='; 
           sudo chmod 777 /dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00;
           ros2 run ros2_head_control head_control;
           exec bash"

# 返回工作空间根目录
cd ../..

# 初始化vr_service（当前终端）
echo "=== 初始化arm_service ==="
cd ./slave/VR_service
colcon build
#这里得交互一下，来确保can0绑定左臂，所以要改init.sh
bash ./script/init.sh

# 启动vr控制节点（独立终端）
gnome-terminal --title="VR CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动vr控制节点 ===';
           ros2 run vr_arm_sync vr_arm_sync_node;
           exec bash"

# 等待2秒确保终端启动完成
sleep 2

# 启动head控制节点（独立终端）
gnome-terminal --title="HEAD CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动head控制节点 ===';
           ros2 run head_servo_controller head_servo_controller;
           exec bash"

# 返回工作空间根目录
cd ../../


# 启动RealSense双摄像头（独立终端）
echo "=== 启动RealSense双摄像头 ==="
cd ./slave/video/camera/librealsense
gnome-terminal --title="REALSENSE CAMERAS" --working-directory="$PWD" -- \
  bash -c "source ./install/setup.bash;
           ros2 launch realsense2_camera rs_dual_camera_launch.py serial_no1:=_130322271114 serial_no2:=_130322271127;
           exec bash"

# 等待2秒确保终端启动完成
sleep 2
cd ../../../..

# 启动ZED摄像头（独立终端）
echo "=== 启动ZED摄像头 ==="
gnome-terminal --title="ZED CAMERA" --working-directory="$PWD" -- \
  bash -c "echo '=== 启动ZED摄像头 ===';
           ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm;
           exec bash"

sleep 2
#启动webrtc
echo "=== 启动webrtc ==="
cd ./slave
cd ./video
cd ./webrtc_pub
colcon build

gnome-terminal --title="WEB RTC" --working-directory="$PWD" -- \
  bash -c "echo '=== 启动WEB RTC ===';
           bash ./start_multi_camera.sh;
           exec bash"


echo "所有模块已启动完成！"
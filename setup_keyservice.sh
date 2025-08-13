#!/bin/bash

echo "这是键盘控制方式的服务端"

#todo:
#检查环境
#检查依赖
#安装依赖
#id号绑定要交互式

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

# 返回工作空间根目录
cd ../..

# 初始化arm_service（当前终端）
echo "=== 初始化arm_service ==="
cd ./slave/key_service/arm_service  
#这里得交互一下，来确保can0绑定左臂，所以要改init.sh
bash ./script/init.sh

# 启动arm控制节点（独立终端）
gnome-terminal --title="ARM CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动arm控制节点 ===';
           ros2 run arm_service ctrl_node;
           exec bash"

# 返回工作空间根目录
cd ../../../

# 启动RealSense双摄像头（独立终端）
echo "=== 启动RealSense双摄像头 ==="
cd ./slave/video/camera/librealsense
gnome-terminal --title="REALSENSE CAMERAS" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           ros2 launch realsense2_camera rs_dual_camera_launch.py serial_no1:=_130322271114 serial_no2:=_130322271127;
           exec bash"
cd ../../../../

# 启动Orbbec摄像头（独立终端）
echo "=== 启动Orbbec摄像头 ==="
cd ./slave/video/camera/orbbecsdk
gnome-terminal --title="ORBBEC CAMERA" --working-directory="$PWD" -- \
  bash -c "colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release；
           source install/setup.bash;
           ros2 launch orbbec_camera astra.launch.py;
           exec bash"
cd ../../../..

# 启动ZED摄像头（独立终端）
echo "=== 启动ZED摄像头 ==="
gnome-terminal --title="ZED CAMERA" --working-directory="$PWD" -- \
  bash -c "echo '=== 启动ZED摄像头 ===';
           ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm;
           exec bash"

# 等待2秒确保终端启动完成

sleep 2

#启动webrtc
echo "=== 启动webrtc ==="
pwd
cd ./slave
cd ./video
cd ./webrtc_pub
colcon build

gnome-terminal --title="WEB RTC" --working-directory="$PWD" -- \
  bash -c "echo '=== 启动WEB RTC ===';
           bash ./start_multi_camera.sh;
           exec bash"


echo "所有模块已启动完成！"
识别特定设备 绑定芯片序列号
输入命令查看制造商信息、产品型号和设备的唯一序列号

ls -l /dev/serial/by-id/
会看到类似这样的输出：

lrwxrwxrwx 1 root root 13 Jul 31 10:00 usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0 -> ../../ttyUSB0
lrwxrwxrwx 1 root root 13 Jul 31 10:00 usb-Arduino__www.arduino.cc__0043_5573530303535180C0C1-if00 -> ../../ttyACM0
为了确保每次访问的都是同一个物理串口设备时，不要使用 /dev/ttyUSB0 或 /dev/ttyACM0。应该使用 /dev/serial/by-id/ 目录下对应的完整符号链接路径。即
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0QI3-if00-port0


最后，由于代码上传不可能上传sdk，所以我们应该专门写一个脚本去作新设备依赖安装，然后在start.sh中选择对应序号去作配置
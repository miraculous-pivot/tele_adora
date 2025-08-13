import serial
import time
import struct

class MOTOR_MG4010E: 
    def __init__(self, port="/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00", baudrate=115200):
        self._ser = None  # 串口对象
        self.serial_opened = False  # 串口状态标志
        
        # 添加yaw和pitch跟踪变量
        self.current_yaw = 0.0      # 当前yaw角度 (度)
        self.current_pitch = 0.0    # 当前pitch角度 (度)
        self.motor_positon_read = 0  # 保持原有变量名的兼容性

        self.init_serial(port, baudrate)
        self.motor_init()

    @property
    def ser(self):
        """提供串口对象的访问接口"""
        return self._ser

    def __del__(self): 
        self.motor_exit()

    # 配置串口参数
    def init_serial(self, port, baudrate):
        try:
            # 初始化串口，设置串口参数
            self._ser = serial.Serial(
                port=port,  # 串口名称
                baudrate=baudrate,  # 波特率
                bytesize=serial.EIGHTBITS,  # 数据位8位
                parity=serial.PARITY_NONE,  # 校验位，无校验位
                stopbits=serial.STOPBITS_ONE,  # 停止位1位
                timeout=0.1  # 超时时间
            )
            self.serial_opened = True
        except Exception as e:
            print("串口初始化失败:", e)
            self.serial_opened = False

    def get_current_angles(self):
        """获取当前的yaw和pitch角度"""
        return {"yaw": self.current_yaw, "pitch": self.current_pitch}
    
    def motor_position_read(self):
        """读取电机位置的方法"""
        # 读取电机1的位置（pitch）
        self.motor_mg4010e_read_multi_loop_angle(1)
        time.sleep(0.01)
        # 读取电机2的位置（yaw）  
        self.motor_mg4010e_read_multi_loop_angle(2)
        time.sleep(0.01)

    def run(self):
        self.motor_position_read()
        uart_buffer_data = self._ser.read_all()  

        if len(uart_buffer_data) < 7:
            print("uart_buffer_data < 7")
            return
        
        # 处理可能的多个数据包
        i = 0
        while i < len(uart_buffer_data):
            # 查找完整的数据包
            if i + 6 < len(uart_buffer_data):
                # 检查是否是位置读取的响应包
                if uart_buffer_data[i] == 0x3E and uart_buffer_data[i+1] == 0x92:
                    motor_id = uart_buffer_data[i+2]  # 电机ID
                    
                    # 获取位置数据 (4字节)
                    if i + 9 < len(uart_buffer_data):
                        position_bytes = uart_buffer_data[i+4:i+8]
                        position_value = int.from_bytes(position_bytes, 'little', signed=True)
                        
                        # 转换位置值为角度
                        angle_degrees = position_value / (10 * 100)  # 转换回角度
                        
                        # 根据电机ID打印对应的yaw或pitch值
                        if motor_id == 1:  # 电机1控制pitch (俯仰)
                            self.current_pitch = angle_degrees
                            print(f"从串口获取 PITCH: {angle_degrees:.2f}° (原始值: {position_value})")
                        elif motor_id == 2:  # 电机2控制yaw (偏航)
                            self.current_yaw = angle_degrees
                            print(f"从串口获取 YAW: {angle_degrees:.2f}° (原始值: {position_value})")
                        else:
                            print(f"从串口获取 电机ID{motor_id}: {angle_degrees:.2f}° (原始值: {position_value})")
                        
                        i += 10  # 跳过这个数据包
                    else:
                        i += 1
                else:
                    i += 1
            else:
                break
        
        # 保留原有的数据包处理逻辑作为备用
        tem_bytes = bytearray()
        tem_bytes.extend(uart_buffer_data[0:min(7, len(uart_buffer_data))]) #高位存低地址
        tem_bytes_str = ' '.join(f"{b:02X}" for b in tem_bytes)
        print(f"HEX: [{tem_bytes_str}]")

        if len(uart_buffer_data) >= 7:
            recives_crc = self.calculate_crc(tem_bytes)
            if recives_crc:
                # 打印十六进制和ASCII格式
                hex_str = ' '.join(f"{b:02X}" for b in uart_buffer_data)
                crc_str = ' '.join(f"{b:02X}" for b in recives_crc)
                ascii_str = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in recives_crc)
                print(f"HEX: [{hex_str}] CRC: [{crc_str}]  ASCII: [{ascii_str}]")

            # 原有的数据解析逻辑 (兼容性保留)
            if uart_buffer_data[0] == 0x01 and uart_buffer_data[1] == 0x03 and uart_buffer_data[2] == 0x04:
                position_bytes = bytearray()
                position_bytes.extend([uart_buffer_data[5], uart_buffer_data[6], uart_buffer_data[3], uart_buffer_data[4]])
                self.motor_positon_read = int.from_bytes(position_bytes, 'big', signed=True)
                print("position:", self.motor_positon_read)

    def motor_init(self):
        print("motor on ...")
        self.motor_mg4010e_On(1)
        time.sleep(0.05)
        print("release brake ...")
        self.motor_mg4010e_set_brake(1, 0x01) #释放刹车
        time.sleep(0.05)
        print("clear error code ...")
        self.motor_mg4010e_clear_error_code(1)
        time.sleep(0.05)

        # 初始化电机2
        self.motor_mg4010e_On(2)
        time.sleep(0.05)
        self.motor_mg4010e_set_brake(2, 0x01)
        time.sleep(0.05)
        self.motor_mg4010e_clear_error_code(2)

    def motor_exit(self):
        self.motor_mg4010e_Off(1)
        time.sleep(0.05)
        self.motor_mg4010e_set_brake(1, 0x00)  
        time.sleep(0.05)
        self.motor_mg4010e_Off(2)
        time.sleep(0.05)
        self.motor_mg4010e_set_brake(2, 0x00)  
        time.sleep(0.05)

        if self._ser and self._ser.is_open:
            self._ser.close() 

    def calculate_crc(self, data: bytes) -> bytes:
        crc_value = 0x00  #初始化
        for byte in data:
            crc_value = crc_value + byte
        return bytes([crc_value & 0xff])
    
    def motor_mg4010e_read_multi_loop_angle(self, motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x92, 0x00, 0x00])
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self._ser.is_open:
            self._ser.open()
        # 发送数据
        self._ser.write(data_array_1)

    def motor_mg4010e_clear_error_code(self, motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9B, 0x00])
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self._ser.is_open:
            self._ser.open()
        # 发送数据
        self._ser.write(data_array_1)
     
    def motor_mg4010e_On(self, motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x88, 0x00, 0x00])
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self._ser.is_open:
            self._ser.open()
        # 发送数据
        self._ser.write(data_array_1)

    def motor_mg4010e_Off(self, motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x81, 0x00, 0x00])
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self._ser.is_open:
            self._ser.open()
        # 发送数据
        self._ser.write(data_array_1)

    # 控制抱闸器的开合，或者读取当前抱闸器的状态。
    # 0x00 启动刹车（抱闸器断电）   0x01 释放刹车（抱闸器通电）    0x10读取抱闸状态
    def motor_mg4010e_set_brake(self, motor_id, cmd):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x8C, 0x00, 0x01])
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        data_array_1.extend([cmd & 0xff])
        data_array_1.extend([cmd & 0xff])
        if not self._ser.is_open:
            self._ser.open()
        # 发送数据
        self._ser.write(data_array_1)

    # 多圈位置控制模式2 
    def motor_mg4010e_set_multi_loop_angle_control2(self, motor_id, angle, speed):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0xA4, 0x00, 0x0C])  
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])

        # int64数据类型 8个字节 + 速度4字节
        data_array_2 = bytearray()
        data_array_2.extend([0x00]*12)         
        data_array_2[0] = angle & 0xff
        data_array_2[1] = (angle>>8)& 0xff
        data_array_2[2] = (angle>>16)& 0xff
        data_array_2[3] = (angle>>24)& 0xff
        data_array_2[4] = (angle>>32)& 0xff
        data_array_2[5] = (angle>>40)& 0xff
        data_array_2[6] = (angle>>48)& 0xff
        data_array_2[7] = (angle>>56)& 0xff
        data_array_2[8] = speed & 0xff
        data_array_2[9] = (speed>>8)& 0xff
        data_array_2[10] = (speed>>16)& 0xff
        data_array_2[11] = (speed>>24)& 0xff
        
        crc_values2 = self.calculate_crc(data_array_2) # calculate crc
        data_array_2.extend([crc_values2[0]])  

        data_array_1.extend(data_array_2)

        if not self._ser.is_open:
            self._ser.open()
        # 发送数据
        self._ser.write(data_array_1)
        
    # 速度控制模式 
    def motor_speed_control(self, motor_id, speed): #单位0.01dps/LSB
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0xA2, 0x00, 0x04])  
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])

        # int32类型 速度4字节
        data_array_2 = bytearray()
        data_array_2.extend([0x00]*4)         
        data_array_2[0] = speed & 0xff
        data_array_2[1] = (speed>>8)& 0xff
        data_array_2[2] = (speed>>16)& 0xff
        data_array_2[3] = (speed>>24)& 0xff
        
        crc_values2 = self.calculate_crc(data_array_2) # calculate crc
        data_array_2.extend([crc_values2[0]])  

        data_array_1.extend(data_array_2)

        if not self._ser.is_open:
            self._ser.open()
        # 发送数据
        self._ser.write(data_array_1)

if __name__ == "__main__":
    app = MOTOR_MG4010E(port="/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00", baudrate=115200)
    while True:
        print("Set 0 position")
        app.motor_mg4010e_set_multi_loop_angle_control2(1, 0*10*100, 30000)
        time.sleep(30)  
        print("Set 270 position") 
        app.motor_mg4010e_set_multi_loop_angle_control2(1, 270*10*100, 30000)
        time.sleep(30)
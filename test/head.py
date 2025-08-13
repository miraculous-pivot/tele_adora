#  MG4010E 电机驱动程序
# 适用于ADORA A2 Pro 机器人头部控制程序
# 机器人水平旋转方向电机ID为2  俯仰方向电机ID为1
# 
import serial
import time
import struct

class MOTOR_MG4010E: 
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.ser = None  # 串口对象
        self.serial_opened = False  # 串口状态标志

        self.init_serial(port, baudrate)
        self.motor_init()

        self.cnt = 1

    def __del__(self): 
        self.motor_exit()

        # 配置串口参数
    def init_serial(self, port, baudrate):
        try:
                # 初始化串口，设置串口参数
            self.ser = serial.Serial(
                port=port,  # 串口名称
                baudrate=baudrate,  # 波特率
                bytesize=serial.EIGHTBITS,  # 数据位8位
                parity=serial.PARITY_NONE,  # 校验位，无校验位
                stopbits=serial.STOPBITS_ONE,  # 停止位1位
                timeout=0.1  # 超时时间
        )
        except Exception as e:
            print("串口初始化失败:", e)

    # 循环读取id1 和 id2 电机的位置
    def run(self):
 
        #while True:
        print("read motor id: ", self.cnt," position")
        if self.cnt <= 1:
            self.motor_mg4010e_read_multi_loop_angle(1)
        else:
            self.motor_mg4010e_read_multi_loop_angle(2)
            self.cnt = 0

        self.cnt = self.cnt + 1
        time.sleep(0.1)

        uart_buffer_data = self.ser.read_all()  
        buffer_len = len(uart_buffer_data)
        if( buffer_len< 6):
            print("uart_buffer_data < 6")
            return
        
        # step1 寻找帧头
        index_i =0
        while(index_i<buffer_len):
            if(uart_buffer_data[index_i] == 0x3E):
                break
            index_i = index_i+1

        #print("index_i: ",index_i)
        # 检查校验信息
        if(index_i+5<=buffer_len): 
            tem_bytes = bytearray()
            tem_bytes.extend(uart_buffer_data[index_i+0:index_i+4])#左闭区 右开区间 #高位存低地址
            tem_bytes_str = ' '.join(f"{b:02X}" for b in tem_bytes)
            #print(f"frame head: [{tem_bytes_str}]")
            recives_crc = self.calculate_crc(tem_bytes)
            if recives_crc:
                # 打印十六进制和ASCII格式
                hex_str = ' '.join(f"{b:02X}" for b in uart_buffer_data)
                crc_str = ' '.join(f"{b:02X}" for b in recives_crc)
                #ascii_str = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in recives_crc)
                #print(f"HEX: [{hex_str}] CRC: [{crc_str}] ]")
            #print("daying:")

            if uart_buffer_data[index_i+4] != recives_crc[0]: #校验不对
                print(["error with calculate crc ",hex(uart_buffer_data[index_i+4]),"   ",recives_crc[0]])
                return
        else:
            print("index_i+5<=len ")
            return
        

        if(uart_buffer_data[index_i] == 0x3E and uart_buffer_data[index_i+1] == 0x92 
            and uart_buffer_data[index_i+3] == 0x08):

            position_bytes = uart_buffer_data[index_i+5:index_i+13]##左闭区 右开区间 高位存低地址
            #print(f"HEX: [{position_bytes}]   ]")
            self.motor_positon_read = int.from_bytes(position_bytes,'little',signed = True)
            id = uart_buffer_data[index_i+2]
            print("mg4010e id ",id,"   position:", self.motor_positon_read)


    def motor_init(self):
        print("motor on id1...")
        self.motor_mg4010e_On(1)
        time.sleep(0.05)
        print("release brake id1...")
        self.motor_mg4010e_set_brake(1,0x01) #释放刹车
        time.sleep(0.05)
        print("clear error code id1...")
        self.motor_mg4010e_clear_error_code(1)
        time.sleep(0.05)

        print("motor on id2...")
        self.motor_mg4010e_On(2)
        time.sleep(0.05)
        print("release brake id2...")
        self.motor_mg4010e_set_brake(2,0x01) #释放刹车
        time.sleep(0.05)
        print("clear error code id2...")
        self.motor_mg4010e_clear_error_code(2)

    def motor_exit(self):
        self.motor_mg4010e_Off(1)
        time.sleep(0.05)
        self.motor_mg4010e_set_brake(1,0x00)  
        time.sleep(0.05)
        self.motor_mg4010e_Off(2)
        time.sleep(0.05)
        self.motor_mg4010e_set_brake(2,0x00)  
        time.sleep(0.05)

        self.ser.close() 

    def calculate_crc(self,data: bytes) -> bytes:
        crc_value = 0x00  #初始化
        for byte in data:
            # 计算索引（修正原代码中的 *puchMsgg++ 错误）
            crc_value = crc_value + byte
        # 返回组合后的CRC值
        return bytes([crc_value & 0xff])
    

    def motor_mg4010e_read_state1(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9A, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_clear_error_code(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9B, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)
     
    def motor_mg4010e_read_state2(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9C, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_read_state3(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9D, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)




    def motor_mg4010e_close(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x80, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_On(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x88, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_Off(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x81, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    # 控制抱闸器的开合，或者读取当前抱闸器的状态。
    # 0x00 启动刹车（抱闸器断电）   0x01 释放刹车（抱闸器通电）    0x10读取抱闸状态
    def motor_mg4010e_set_brake(self,motor_id,cmd):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x8C, 0x00, 0x01])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        data_array_1.extend([cmd & 0xff])
        data_array_1.extend([cmd & 0xff])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)


    # 10. 转矩闭环控制
    # 主机发送该命令以控制电机的转矩电流输出，控制值 iqControl 为 int16_t 类型，数值范围-2048~ 2048，
    # 对应 MF 电机实际转矩电流范围-16.5A~16.5A，
    # 对应 MG 电机实际转矩电流范围-33A~33A，母线电流和电机的实际扭矩因不同电机而异。
    def motor_mg4010e_set_torque(self,motor_id,torque):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0xA1, 0x00, 0x02])  
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])

        data_array_2 = bytearray()
        data_array_2.extend([0x00, 0x00])  
        if(torque > 2048):
            torque = 2048  
        elif(torque < -2048):
            torque = -2048
        print('torque: ',torque)
        data_array_2[0] = torque & 0xff
        data_array_2[1] = (torque>>8)& 0xff
        crc_values2 = self.calculate_crc(data_array_2) # calculate crc
        data_array_2.extend([crc_values2[0]])  

        data_array_1.extend(data_array_2)
        hex_str = ' '.join(f'{byte:02x}' for byte in data_array_1)
        print("data_array_1",hex_str)  # 输出: '01 02 03' [4](@ref)
        #print("data_array_2: ",data_array_2)
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    # 12.多圈位置控制模式1
    # 控制值 angleControl 为 int64_t 类型，对应实际位置为 0.01degree/LSB，即 36000 代表 360°，
    # 电机转动方向由目标位置和当前位置的差值决定。
    # 测试中从0转到270为逆时针方向，从270度转到0为顺时针方向
    def motor_mg4010e_set_multi_loop_angle_control1(self,motor_id,angle):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0xA3, 0x00, 0x08])  
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])

        print('set multi loop  angle: ',angle)
        # int64数据类型 8个字节，最后添加一个校验位
        data_array_2 = bytearray()
        data_array_2.extend([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])         
        data_array_2[0] = angle & 0xff
        data_array_2[1] = (angle>>8)& 0xff
        data_array_2[2] = (angle>>16)& 0xff
        data_array_2[3] = (angle>>24)& 0xff
        data_array_2[4] = (angle>>32)& 0xff
        data_array_2[5] = (angle>>40)& 0xff
        data_array_2[6] = (angle>>48)& 0xff
        data_array_2[7] = (angle>>56)& 0xff
        crc_values2 = self.calculate_crc(data_array_2) # calculate crc
        data_array_2.extend([crc_values2[0]])  

        data_array_1.extend(data_array_2)

        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    # 13.多圈位置控制模式2 
    # 注意该函数没有考虑减速比， mg4010e 电机减速比为10
    #控制值 angle 为 int64_t 类型，对应实际位置为 0.01degree/LSB，即 36000 代表 360°，
    #控制值 speed 限制了电机转动的最大速度，为 uint32_t 类型，对应实际转速 0.01dps/LSB，即 36000 代表 360dps。。
    def motor_mg4010e_set_multi_loop_angle_control2(self,motor_id,angle,speed):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0xA4, 0x00, 0x0C])  
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])

        print('set multi loop  angle and speed: ',angle,'    ',speed)
        # int64数据类型 8个字节，最后添加一个校验位
        data_array_2 = bytearray()
        data_array_2.extend([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])         
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

        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    # 14.主机发送该命令以控制电机的角度（单圈1角度）， 
    # 注意该电机的减速比1:10，即如何按表格中的命令发送36000，电机转动360度，但是实际只有外转子只转动了36度
    # 控制值 dir 设置电机转动的方向，为 uint8_t 类型， 0x00 代表顺时针， 0x01 代表逆时针
    # 控制值 angle 为 uint16_t 类型，数值范围 0~35999，对应实际位置为 0.01degree/LSB，
    # 即实际角度范围 0°~359.99°。
    def motor_mg4010e_set_single_loop_angle_control1(self,motor_id,dir,angle):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0xA5, 0x00, 0x04])  
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])

        data_array_2 = bytearray()
        data_array_2.extend([0x00, 0x00, 0x00, 0x00])  
        if(dir > 0):
            data_array_2[0] = 0x00 #  0x00 代表顺时针， 0x01 代表逆时针
        elif(dir < 0):
            data_array_2[0] = 0x01
        
        if(angle < 0):
            angle = 0
        elif(angle > 35999):
            angle = 35999   

        print('angle: ',angle)
        data_array_2[1] = angle & 0xff
        data_array_2[2] = (angle>>8)& 0xff

        crc_values2 = self.calculate_crc(data_array_2) # calculate crc
        data_array_2.extend([crc_values2[0]])  

        data_array_1.extend(data_array_2)
        hex_str = ' '.join(f'{byte:02x}' for byte in data_array_1)
        print("data_array_1",hex_str)  # 输出: '01 02 03' [4](@ref)
        #print("data_array_2: ",data_array_2)
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)
 
 




    # 21.设置当前位置为0点（写入到ROM）
    # 设置电机当前位置的编码器原始值作为电机上电后的初始零点
    def motor_mg4010e_set_oringin(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x19, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)


    # 22.读取多圈角度命令
    def motor_mg4010e_read_multi_loop_angle(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x92, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    # 23.清除电机圈数信息命令
    def motor_mg4010e_clear_multi_loop_angle(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x93, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)



    # 24.读取单圈角度命令
    def motor_mg4010e_read_single_loop_angle(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x94, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)


if __name__ == "__main__":
    app = MOTOR_MG4010E(port="/dev/ttyACM0", baudrate=115200)

    app.motor_mg4010e_set_torque(1,0)#0力矩模式
    time.sleep(1)
    app.motor_mg4010e_set_torque(2,0)#0力矩模式
    time.sleep(1)

    while True:
        app.run()
        time.sleep(0.1)


    #位置控制模式
    app.motor_mg4010e_set_multi_loop_angle_control2(1,0*10*100,30000)
    time.sleep(30)
    #app.run()
    while True:
        print("set 270")
        # 270 度
        #app.motor_mg4010e_set_multi_loop_angle_control1(1,270*10*100)
        # 乘10是减速比是10，乘100是下发的单位为0.01度
        app.motor_mg4010e_set_multi_loop_angle_control2(1,270*10*100,30000)
        time.sleep(30)  
        print("set 0") 
        #app.motor_mg4010e_set_multi_loop_angle_control1(1,0*10*100)
        app.motor_mg4010e_set_multi_loop_angle_control2(1,0*10*100,30000)
        time.sleep(30)  

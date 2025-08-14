# bimanual_teleop.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import Bool, UInt32
import sys, select, termios, tty

# 打印的说明信息
msg = """
简化双臂机器人遥控 - 仅使用字母键控制
====================================================
操作理念：小写字母=正方向，大写字母=负方向

左手区域（左臂控制 + 底盘移动）：
    左臂6DOF控制：
    q/Q: +X/-X (前进/后退)
    w/W: +Y/-Y (左/右)  
    e/E: +Z/-Z (上/下)
    r/R: +RX/-RX (roll)
    t/T: +RY/-RY (pitch)  
    y/Y: +RZ/-RZ (yaw)
    u/U: 夹爪开/关
    
    底盘移动：
    v/V: 前进/后退
    b/B: 左转/右转

右手区域（右臂控制 + 云台控制）：
    右臂6DOF控制：
    a/A: +X/-X (前进/后退)
    s/S: +Y/-Y (左/右)
    d/D: +Z/-Z (上/下)  
    f/F: +RX/-RX (roll)
    g/G: +RY/-RY (pitch)
    h/H: +RZ/-RZ (yaw)
    j/J: 夹爪开/关
    
    云台控制：
    n/N: 抬头/低头
    m/M: 左转/右转

通用控制：
    c/C: 吸盘开/关
    z/Z: 升降上/下
    
空格键 或 ESC: 紧急停止并退出
====================================================
"""

# 简化的键位映射 - 只使用字母键，大小写控制方向
# 格式: (x, y, z, rx, ry, rz)

# 左臂控制键位（左手区域）
left_arm_keys = {
    # 位置控制 (XYZ)
    'q': (1, 0, 0, 0, 0, 0),   'Q': (-1, 0, 0, 0, 0, 0),   # X轴 前进/后退
    'w': (0, 1, 0, 0, 0, 0),   'W': (0, -1, 0, 0, 0, 0),   # Y轴 左/右
    'e': (0, 0, 1, 0, 0, 0),   'E': (0, 0, -1, 0, 0, 0),   # Z轴 上/下
    # 姿态控制 (RxRyRz)
    'r': (0, 0, 0, 1, 0, 0),   'R': (0, 0, 0, -1, 0, 0),   # Roll
    't': (0, 0, 0, 0, 1, 0),   'T': (0, 0, 0, 0, -1, 0),   # Pitch
    'y': (0, 0, 0, 0, 0, 1),   'Y': (0, 0, 0, 0, 0, -1),   # Yaw
}

# 右臂控制键位（右手区域）
right_arm_keys = {
    # 位置控制 (XYZ)
    'a': (1, 0, 0, 0, 0, 0),   'A': (-1, 0, 0, 0, 0, 0),   # X轴 前进/后退
    's': (0, 1, 0, 0, 0, 0),   'S': (0, -1, 0, 0, 0, 0),   # Y轴 左/右
    'd': (0, 0, 1, 0, 0, 0),   'D': (0, 0, -1, 0, 0, 0),   # Z轴 上/下
    # 姿态控制 (RxRyRz)
    'f': (0, 0, 0, 1, 0, 0),   'F': (0, 0, 0, -1, 0, 0),   # Roll
    'g': (0, 0, 0, 0, 1, 0),   'G': (0, 0, 0, 0, -1, 0),   # Pitch
    'h': (0, 0, 0, 0, 0, 1),   'H': (0, 0, 0, 0, 0, -1),   # Yaw
}

# 夹爪控制
left_gripper_keys = ['u', 'U']   # 左臂夹爪
right_gripper_keys = ['j', 'J']  # 右臂夹爪

# 底盘控制键位
chassis_keys = {
    'v': 'forward',    'V': 'backward',   # 前进/后退
    'b': 'turn_left',  'B': 'turn_right', # 左转/右转
}

# 云台控制键位
head_keys = {
    'n': 'up',      'N': 'down',    # 抬头/低头
    'm': 'left',    'M': 'right',   # 左转/右转
}

# 通用控制
suction_keys = ['c', 'C']           # 吸盘开关
lifting_keys = {'z': 'up', 'Z': 'down'}  # 升降控制


def get_key(settings):
    """获取单个按键，简化版本"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class BimanualTeleopNode(Node):
    def __init__(self):
        super().__init__('bimanual_teleop_node')
        
        # 创建发布者
        self.left_arm_pub = self.create_publisher(Pose, '/left_arm/pose_cmd', 10)
        self.left_gripper_pub = self.create_publisher(Bool, '/left_arm/gripper_cmd', 10)
        self.right_arm_pub = self.create_publisher(Pose, '/right_arm/pose_cmd', 10)
        self.right_gripper_pub = self.create_publisher(Bool, '/right_arm/gripper_cmd', 10)
        
        # 云台控制发布者
        self.head_control_pub = self.create_publisher(Point, 'adora_robot/head/servo_cmd', 10)
        
        # 底盘控制发布者
        self.chassis_control_pub = self.create_publisher(Twist, '/dt/velocity_ctrl', 10)
        
        # 吸盘控制发布者
        self.suction_pump_pub = self.create_publisher(Bool, '/adora_robot/suction_pump/cmd', 10)
        
        # 升降电机发布者和订阅者
        self.lifting_motor_pub = self.create_publisher(UInt32, '/adora/lifting_motor/cmd', 10)
        self.lifting_motor_sub = self.create_subscription(
            UInt32, 
            '/adora/lifting_motor/states', 
            self.lifting_motor_state_callback, 
            10
        )
        
        # 创建定时器用于持续升降控制
        self.lifting_control_timer = self.create_timer(0.00001, self.lifting_control_callback)  # 100Hz
        
        # 速度参数
        self.pos_speed = self.declare_parameter('pos_speed', 0.01).value # m/key_press
        self.rot_speed = self.declare_parameter('rot_speed', 0.05).value # rad/key_press
        self.lifting_step = self.declare_parameter('lifting_step', 10).value # mm/key_press
        self.max_lifting_height = self.declare_parameter('max_lifting_height', 700).value # mm
        self.lifting_speed_mms = self.declare_parameter('lifting_speed_mms', 5000).value # mm/s 升降速度
        
        # 云台控制参数
        self.head_pitch_speed = self.declare_parameter('head_pitch_speed', 0.2).value # rad/key_press (增大增量)
        self.head_yaw_speed = self.declare_parameter('head_yaw_speed', 0.2).value # rad/key_press (增大增量)
        
        # 底盘控制参数
        self.chassis_linear_speed = self.declare_parameter('chassis_linear_speed', 0.5).value # m/s
        self.chassis_angular_speed = self.declare_parameter('chassis_angular_speed', 0.5).value # rad/s
        
        # 初始化双臂状态变量
        self.left_pose_delta = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        self.right_pose_delta = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        
        self.left_gripper_open = True
        self.right_gripper_open = True 
        
        # 云台状态变量 - 改为累积位置
        self.head_pitch_position = 0.0  # 当前目标pitch位置
        self.head_yaw_position = 0.0    # 当前目标yaw位置
        self.head_pitch_delta = 0.0     # 本次控制的增量
        self.head_yaw_delta = 0.0       # 本次控制的增量 
        
        # 底盘状态变量
        self.chassis_linear_vel = 0.0
        self.chassis_angular_vel = 0.0
        
        # 吸盘状态变量
        self.suction_pump_on = False 
        
        # 升降电机状态
        self.current_lifting_height = None  # 初始化为None，等待从话题读取真实值
        self.target_lifting_height = None   # 初始化为None
        self.lifting_motor_ready = False    # 标记是否已读取到初始位置
        
        # 切换式升降控制状态
        self.lifting_up_active = False    # 持续上升状态
        self.lifting_down_active = False  # 持续下降状态
        # 计算每个控制周期的升降距离 (100Hz -> 0.01秒/周期)
        self.lifting_speed = self.lifting_speed_mms * 0.01  # mm per control cycle
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Bimanual Teleop Node with Lifting Motor Control Started.")
        self.get_logger().info("Waiting for lifting motor initial position...")
        self.get_logger().info(msg)
        
        self.run()

    def lifting_motor_state_callback(self, msg):
        """升降电机状态回调函数"""
        self.current_lifting_height = msg.data
        
        # 如果是第一次接收到位置数据，初始化目标位置
        if not self.lifting_motor_ready:
            self.target_lifting_height = self.current_lifting_height
            self.lifting_motor_ready = True
            self.get_logger().info(f"Lifting motor initialized at {self.current_lifting_height}mm")

    def lifting_control_callback(self):
        """定时器回调函数，用于持续升降控制"""
        if not self.lifting_motor_ready:
            return
            
        if self.lifting_up_active:
            # 持续上升
            new_height = self.current_lifting_height + self.lifting_speed
            if new_height <= self.max_lifting_height:
                self.target_lifting_height = new_height
                self.publish_lifting_command()
            else:
                # 到达最大高度，自动停止
                self.lifting_up_active = False
                self.get_logger().info("Reached maximum height, stopping upward movement")
                
        elif self.lifting_down_active:
            # 持续下降
            new_height = self.current_lifting_height - self.lifting_speed
            if new_height >= 0:
                self.target_lifting_height = new_height
                self.publish_lifting_command()
            else:
                # 到达最低位置，自动停止
                self.lifting_down_active = False
                self.get_logger().info("Reached minimum height, stopping downward movement")

    def publish_lifting_command(self):
        """发布升降电机命令"""
        if self.target_lifting_height is not None:
            lifting_cmd = UInt32()
            lifting_cmd.data = int(self.target_lifting_height)
            self.lifting_motor_pub.publish(lifting_cmd)

    def run(self):
        try:
            # 等待升降电机初始位置
            self.get_logger().info("Waiting for lifting motor initial position...")
            while rclpy.ok() and not self.lifting_motor_ready:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not rclpy.ok():
                return
                
            self.get_logger().info("All systems ready! Starting teleop control...")
            
            while rclpy.ok():
                # 处理ROS消息回调
                rclpy.spin_once(self, timeout_sec=0.001)
                
                key = get_key(self.settings)

                # 默认将所有速度目标清零
                self.left_pose_delta = {k: 0.0 for k in self.left_pose_delta}
                self.right_pose_delta = {k: 0.0 for k in self.right_pose_delta}
                
                # 清零底盘速度
                self.chassis_linear_vel = 0.0
                self.chassis_angular_vel = 0.0
                
                # 标记是否有云台控制命令
                head_control_active = False

                # --- 根据按键更新状态 ---
                # 左臂控制
                if key in left_arm_keys:
                    deltas = left_arm_keys[key]
                    self.left_pose_delta['x'] = deltas[0]
                    self.left_pose_delta['y'] = deltas[1]
                    self.left_pose_delta['z'] = deltas[2]
                    self.left_pose_delta['rx'] = deltas[3]
                    self.left_pose_delta['ry'] = deltas[4]
                    self.left_pose_delta['rz'] = deltas[5]

                # 左臂夹爪控制
                elif key in left_gripper_keys:
                    self.left_gripper_open = not self.left_gripper_open
                
                # 右臂控制
                elif key in right_arm_keys:
                    deltas = right_arm_keys[key]
                    self.right_pose_delta['x'] = deltas[0]
                    self.right_pose_delta['y'] = deltas[1]
                    self.right_pose_delta['z'] = deltas[2]
                    self.right_pose_delta['rx'] = deltas[3]
                    self.right_pose_delta['ry'] = deltas[4]
                    self.right_pose_delta['rz'] = deltas[5]

                # 右臂夹爪控制
                elif key in right_gripper_keys:
                    self.right_gripper_open = not self.right_gripper_open
                
                # 云台控制
                elif key in head_keys:
                    direction = head_keys[key]
                    head_control_active = True
                    if direction == 'up':
                        self.head_pitch_position += self.head_pitch_speed
                        self.head_pitch_delta = self.head_pitch_position
                        self.head_yaw_delta = self.head_yaw_position
                    elif direction == 'down':
                        self.head_pitch_position -= self.head_pitch_speed
                        self.head_pitch_delta = self.head_pitch_position
                        self.head_yaw_delta = self.head_yaw_position
                    elif direction == 'left':
                        self.head_yaw_position += self.head_yaw_speed
                        self.head_pitch_delta = self.head_pitch_position
                        self.head_yaw_delta = self.head_yaw_position
                    elif direction == 'right':
                        self.head_yaw_position -= self.head_yaw_speed
                        self.head_pitch_delta = self.head_pitch_position
                        self.head_yaw_delta = self.head_yaw_position
                
                # 底盘控制
                elif key in chassis_keys:
                    direction = chassis_keys[key]
                    if direction == 'forward':
                        self.chassis_linear_vel = self.chassis_linear_speed
                    elif direction == 'backward':
                        self.chassis_linear_vel = -self.chassis_linear_speed
                    elif direction == 'turn_left':
                        self.chassis_angular_vel = self.chassis_angular_speed
                    elif direction == 'turn_right':
                        self.chassis_angular_vel = -self.chassis_angular_speed
                
                # 吸盘控制
                elif key in suction_keys:
                    self.suction_pump_on = not self.suction_pump_on
                    if self.suction_pump_on:
                        self.get_logger().info("Suction pump turned ON")
                    else:
                        self.get_logger().info("Suction pump turned OFF")
                
                # 升降电机控制
                elif key in lifting_keys:
                    direction = lifting_keys[key]
                    if self.lifting_motor_ready and self.current_lifting_height is not None:
                        if direction == 'up':
                            if self.lifting_up_active:
                                # 当前正在上升，按键停止
                                self.lifting_up_active = False
                                self.get_logger().info("Stopping upward movement")
                            else:
                                # 当前未在上升，开始上升
                                self.lifting_down_active = False  # 确保下降停止
                                self.lifting_up_active = True
                                self.get_logger().info("Starting continuous upward movement")
                        elif direction == 'down':
                            if self.lifting_down_active:
                                # 当前正在下降，按键停止
                                self.lifting_down_active = False
                                self.get_logger().info("Stopping downward movement")
                            else:
                                # 当前未在下降，开始下降
                                self.lifting_up_active = False  # 确保上升停止
                                self.lifting_down_active = True
                                self.get_logger().info("Starting continuous downward movement")
                    else:
                        self.get_logger().warn("Lifting motor not ready yet, waiting for initial position...")
                
                # 退出条件 - 空格键或ESC键
                elif key == ' ' or key == '\x1b':
                    break
                
                # --- 发布左臂消息 ---
                left_pose_msg = Pose()
                left_pose_msg.position.x = self.left_pose_delta['x'] * self.pos_speed
                left_pose_msg.position.y = self.left_pose_delta['y'] * self.pos_speed
                left_pose_msg.position.z = self.left_pose_delta['z'] * self.pos_speed
                # For simplicity, we send orientation changes as Euler angles in the orientation fields
                # The subscriber will need to interpret this correctly
                left_pose_msg.orientation.x = self.left_pose_delta['rx'] * self.rot_speed
                left_pose_msg.orientation.y = self.left_pose_delta['ry'] * self.rot_speed
                left_pose_msg.orientation.z = self.left_pose_delta['rz'] * self.rot_speed
                left_pose_msg.orientation.w = 1.0 # Flag to indicate this is a delta
                self.left_arm_pub.publish(left_pose_msg)
                
                left_gripper_cmd = Bool()
                left_gripper_cmd.data = self.left_gripper_open
                self.left_gripper_pub.publish(left_gripper_cmd)

                # --- 发布右臂消息 ---
                right_pose_msg = Pose()
                right_pose_msg.position.x = self.right_pose_delta['x'] * self.pos_speed
                right_pose_msg.position.y = self.right_pose_delta['y'] * self.pos_speed
                right_pose_msg.position.z = self.right_pose_delta['z'] * self.pos_speed
                right_pose_msg.orientation.x = self.right_pose_delta['rx'] * self.rot_speed
                right_pose_msg.orientation.y = self.right_pose_delta['ry'] * self.rot_speed
                right_pose_msg.orientation.z = self.right_pose_delta['rz'] * self.rot_speed
                right_pose_msg.orientation.w = 1.0 # Flag to indicate this is a delta
                self.right_arm_pub.publish(right_pose_msg)

                right_gripper_cmd = Bool()
                right_gripper_cmd.data = self.right_gripper_open
                self.right_gripper_pub.publish(right_gripper_cmd)
                
                # --- 发布云台控制消息 ---
                # 只在有控制命令时才发送，避免持续发送零值造成抖动
                if head_control_active:
                    head_cmd = Point()
                    head_cmd.x = self.head_pitch_delta  # pitch（俯仰）
                    head_cmd.y = self.head_yaw_delta    # yaw（偏航）
                    head_cmd.z = 0.0  # 未使用
                    self.head_control_pub.publish(head_cmd)
                
                # 始终显示当前累积位置
                head_display_pitch = self.head_pitch_position
                head_display_yaw = self.head_yaw_position
                
                # --- 发布底盘控制消息 ---
                chassis_cmd = Twist()
                chassis_cmd.linear.x = self.chassis_linear_vel   # 前进后退速度
                chassis_cmd.linear.y = 0.0  # 未使用
                chassis_cmd.linear.z = 0.0  # 未使用
                chassis_cmd.angular.x = 0.0  # 未使用
                chassis_cmd.angular.y = 0.0  # 未使用
                chassis_cmd.angular.z = self.chassis_angular_vel  # 旋转速度
                self.chassis_control_pub.publish(chassis_cmd)
                
                # --- 发布吸盘控制消息 ---
                suction_cmd = Bool()
                suction_cmd.data = self.suction_pump_on
                self.suction_pump_pub.publish(suction_cmd)
                
                # --- 打印状态 ---
                left_g_str = "OPEN" if self.left_gripper_open else "CLOSED"
                right_g_str = "OPEN" if self.right_gripper_open else "CLOSED"
                
                # 升降电机状态显示
                if self.lifting_motor_ready:
                    # 显示运动状态
                    movement_status = ""
                    if self.lifting_up_active:
                        movement_status = "↑ LIFTING UP"
                    elif self.lifting_down_active:
                        movement_status = "↓ LIFTING DOWN"
                    else:
                        movement_status = "⏸ STOPPED"
                    
                    lifting_status = f"Current:{self.current_lifting_height}mm | {movement_status} | Speed:{self.lifting_speed_mms}mm/s | Max:{self.max_lifting_height}mm"
                else:
                    lifting_status = "Waiting for initial position..."
                
                status_str = (f"Left Arm Vel: [x:{left_pose_msg.position.x:+.2f}, y:{left_pose_msg.position.y:+.2f}, z:{left_pose_msg.position.z:+.2f}, "
                              f"rx:{left_pose_msg.orientation.x:+.2f}, ry:{left_pose_msg.orientation.y:+.2f}, rz:{left_pose_msg.orientation.z:+.2f}] | Gripper: {left_g_str}\n"
                              f"Right Arm Vel:[x:{right_pose_msg.position.x:+.2f}, y:{right_pose_msg.position.y:+.2f}, z:{right_pose_msg.position.z:+.2f}, "
                              f"rx:{right_pose_msg.orientation.x:+.2f}, ry:{right_pose_msg.orientation.y:+.2f}, rz:{right_pose_msg.orientation.z:+.2f}] | Gripper: {right_g_str}\n"
                              f"Head Control: [Pitch:{head_display_pitch:+.3f}, Yaw:{head_display_yaw:+.3f}]\n"
                              f"Chassis Control: [Linear:{chassis_cmd.linear.x:+.2f} m/s, Angular:{chassis_cmd.angular.z:+.2f} rad/s]\n"
                              f"Suction Pump: {'ON' if suction_cmd.data else 'OFF'}\n"
                              f"Lifting Motor: {lifting_status}")
                print(f'\r{status_str}', end='')

        finally:
            # 退出前停止所有运动
            self.lifting_up_active = False
            self.lifting_down_active = False
            self.left_arm_pub.publish(Pose())
            self.right_arm_pub.publish(Pose())
            # 停止云台运动
            self.head_control_pub.publish(Point())
            # 停止底盘运动
            self.chassis_control_pub.publish(Twist())
            # 停止吸盘
            stop_suction_cmd = Bool()
            stop_suction_cmd.data = False
            self.suction_pump_pub.publish(stop_suction_cmd)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nAll movements stopped. Node shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = BimanualTeleopNode()
    # run() 方法已经包含了自己的spin循环，所以不需要额外的spin
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# bimanual_teleop.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import Bool, UInt32
import sys, select, termios, tty

# 打印的说明信息
msg = """
Bimanual (Two-Armed) Robotic Teleop - 6-DOF Control + Lifting Motor + Head Control                 # 云台控制 - 数字键1,3,5,7控制
                elif key in head_control_keys:
                    direction = head_control_keys[key]
                    head_control_active = True
                    if direction == 'up':
                        self.head_pitch_delta = self.head_pitch_speed  # 抬头
                        self.head_yaw_delta = 0.0
                    elif direction == 'down':
                        self.head_pitch_delta = -self.head_pitch_speed  # 低头
                        self.head_yaw_delta = 0.0
                    elif direction == 'left':
                        self.head_pitch_delta = 0.0
                        self.head_yaw_delta = self.head_yaw_speed  # 向左转
                    elif direction == 'right':
                        self.head_pitch_delta = 0.0
                        self.head_yaw_delta = -self.head_yaw_speed  # 向右转ontrol + Suction Pump
-------------------------------------------------------------------------------------------------------------------
Left Hand:
    w/s: +X / -X (forward/backward)
    a/d: +Y / -Y (left/right)
    q/e: +Z / -Z (up/down)
    r/f: +RX / -RX (roll)
    t/g: +RY / -RY (pitch)
    y/h: +RZ / -RZ (yaw)
    v:   toggle gripper

Right Hand:
    i/k: +X / -X (forward/backward)
    j/l: +Y / -Y (left/right)
    u/o: +Z / -Z (up/down)
    p/;: +RX / -RX (roll)
    '[: +RY / -RY (pitch)
    ]': +RZ / -RZ (yaw)
    m:   toggle gripper

Head Control (Gimbal):
    n: +Pitch (look up)
    b: -Pitch (look down)
    ,: +Yaw (turn left)
    .: -Yaw (turn right)

Chassis Control:
    9: Forward (前进)
    0: Backward (后退)  
    -: Turn Left (左转)
    =: Turn Right (右转)

Suction Pump Control:
    c: Toggle suction pump (on/off)

Lifting Motor (Toggle Mode):
    z:   toggle lift up (press once to start, press again to stop)
    x:   toggle lift down (press once to start, press again to stop)

space key or 'esc': force stop and quit
"""

# 键位映射: (x, y, z, rx, ry, rz)
key_bindings = {
    # Left Hand
    'w': (1, 0, 0, 0, 0, 0), 's': (-1, 0, 0, 0, 0, 0),
    'a': (0, 1, 0, 0, 0, 0), 'd': (0, -1, 0, 0, 0, 0),
    'q': (0, 0, 1, 0, 0, 0), 'e': (0, 0, -1, 0, 0, 0),
    'r': (0, 0, 0, 1, 0, 0), 'f': (0, 0, 0, -1, 0, 0),
    't': (0, 0, 0, 0, 1, 0), 'g': (0, 0, 0, 0, -1, 0),
    'y': (0, 0, 0, 0, 0, 1), 'h': (0, 0, 0, 0, 0, -1),
    # Right Hand
    'i': (1, 0, 0, 0, 0, 0), 'k': (-1, 0, 0, 0, 0, 0),
    'j': (0, 1, 0, 0, 0, 0), 'l': (0, -1, 0, 0, 0, 0),
    'u': (0, 0, 1, 0, 0, 0), 'o': (0, 0, -1, 0, 0, 0),
    'p': (0, 0, 0, 1, 0, 0), ';': (0, 0, 0, -1, 0, 0),
    '[': (0, 0, 0, 0, 1, 0), ']': (0, 0, 0, 0, -1, 0),
    "'": (0, 0, 0, 0, 0, 1), '\\':(0, 0, 0, 0, 0, -1),
}

left_hand_keys = ['w', 's', 'a', 'd', 'q', 'e', 'r', 'f', 't', 'g', 'y', 'h']
right_hand_keys = ['i', 'k', 'j', 'l', 'u', 'o', 'p', ';', '[', ']', "'", '\\']
left_gripper_key = 'v'
right_gripper_key = 'm'
lifting_up_key = 'z'
lifting_down_key = 'x'
suction_pump_key = 'c'

# 云台控制键位 - 使用方向键式布局的字母键
head_control_keys = {
    'n': 'up',      # n: 抬头（+Pitch）
    'b': 'down',    # b: 低头（-Pitch）  
    ',': 'left',    # ,: 向左转（+Yaw）
    '.': 'right'    # .: 向右转（-Yaw）
}

# 底盘控制键位 - 使用右上角按键（避免与手臂控制冲突）
chassis_control_keys = {
    '9': 'forward',     # 9: 前进
    '0': 'backward',    # 0: 后退
    '-': 'turn_left',   # -: 左转
    '=': 'turn_right'   # =: 右转
}


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

                # 调试：打印所有按键
                if key:
                    ascii_val = ord(key)
                    self.get_logger().info(f"Key: '{key}' ASCII: {ascii_val}")
                    
                    # 特别检查云台控制键
                    if key in ['n', 'b', ',', '.']:
                        self.get_logger().info(f"Detected head control key: {key}")
                    elif key == '\x1b':
                        self.get_logger().info("ESC key detected - this will exit the program")

                # 默认将所有速度目标清零
                self.left_pose_delta = {k: 0.0 for k in self.left_pose_delta}
                self.right_pose_delta = {k: 0.0 for k in self.right_pose_delta}
                
                # 清零底盘速度
                self.chassis_linear_vel = 0.0
                self.chassis_angular_vel = 0.0
                
                # 标记是否有云台控制命令
                head_control_active = False

                # --- 根据按键更新状态 ---
                if key in left_hand_keys:
                    deltas = key_bindings[key]
                    self.left_pose_delta['x'] = deltas[0]
                    self.left_pose_delta['y'] = deltas[1]
                    self.left_pose_delta['z'] = deltas[2]
                    self.left_pose_delta['rx'] = deltas[3]
                    self.left_pose_delta['ry'] = deltas[4]
                    self.left_pose_delta['rz'] = deltas[5]

                elif key == left_gripper_key:
                    self.left_gripper_open = not self.left_gripper_open
                
                elif key in right_hand_keys:
                    deltas = key_bindings[key]
                    self.right_pose_delta['x'] = deltas[0]
                    self.right_pose_delta['y'] = deltas[1]
                    self.right_pose_delta['z'] = deltas[2]
                    self.right_pose_delta['rx'] = deltas[3]
                    self.right_pose_delta['ry'] = deltas[4]
                    self.right_pose_delta['rz'] = deltas[5]

                elif key == right_gripper_key:
                    self.right_gripper_open = not self.right_gripper_open
                
                # 云台控制 - 数字键1,3,5,7控制
                elif key in head_control_keys:
                    direction = head_control_keys[key]
                    head_control_active = True
                    self.get_logger().info(f"Head control key pressed: {key} -> {direction}")  # 调试信息
                    if direction == 'up':
                        self.head_pitch_position += self.head_pitch_speed  # 累积增量
                        self.head_pitch_delta = self.head_pitch_position
                        self.head_yaw_delta = self.head_yaw_position
                        self.get_logger().info(f"Up: pitch_pos={self.head_pitch_position}, pitch_delta={self.head_pitch_delta}")
                    elif direction == 'down':
                        self.head_pitch_position -= self.head_pitch_speed  # 累积增量
                        self.head_pitch_delta = self.head_pitch_position
                        self.head_yaw_delta = self.head_yaw_position
                        self.get_logger().info(f"Down: pitch_pos={self.head_pitch_position}, pitch_delta={self.head_pitch_delta}")
                    elif direction == 'left':
                        self.head_yaw_position += self.head_yaw_speed      # 累积增量
                        self.head_pitch_delta = self.head_pitch_position
                        self.head_yaw_delta = self.head_yaw_position
                        self.get_logger().info(f"Left: yaw_pos={self.head_yaw_position}, yaw_delta={self.head_yaw_delta}")
                    elif direction == 'right':
                        self.head_yaw_position -= self.head_yaw_speed      # 累积增量
                        self.head_pitch_delta = self.head_pitch_position
                        self.head_yaw_delta = self.head_yaw_position
                        self.get_logger().info(f"Right: yaw_pos={self.head_yaw_position}, yaw_delta={self.head_yaw_delta}")
                
                # 底盘控制 - 数字键8,2,4,6控制
                elif key in chassis_control_keys:
                    direction = chassis_control_keys[key]
                    if direction == 'forward':
                        self.chassis_linear_vel = self.chassis_linear_speed  # 前进
                    elif direction == 'backward':
                        self.chassis_linear_vel = -self.chassis_linear_speed  # 后退
                    elif direction == 'turn_left':
                        self.chassis_angular_vel = self.chassis_angular_speed  # 左转
                    elif direction == 'turn_right':
                        self.chassis_angular_vel = -self.chassis_angular_speed  # 右转
                
                # 吸盘控制 - c键切换
                elif key == suction_pump_key:
                    self.suction_pump_on = not self.suction_pump_on
                    if self.suction_pump_on:
                        self.get_logger().info("Suction pump turned ON")
                    else:
                        self.get_logger().info("Suction pump turned OFF")
                
                # 升降电机控制 - 切换式控制
                elif key == lifting_up_key:
                    if self.lifting_motor_ready and self.current_lifting_height is not None:
                        if self.lifting_up_active:
                            # 当前正在上升，按键停止
                            self.lifting_up_active = False
                            self.get_logger().info("Stopping upward movement")
                        else:
                            # 当前未在上升，开始上升
                            self.lifting_down_active = False  # 确保下降停止
                            self.lifting_up_active = True
                            self.get_logger().info("Starting continuous upward movement")
                    else:
                        self.get_logger().warn("Lifting motor not ready yet, waiting for initial position...")
                
                elif key == lifting_down_key:
                    if self.lifting_motor_ready and self.current_lifting_height is not None:
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
                
                # 退出条件 - 只有纯ESC键或空格键才退出
                elif key == ' ' or key == '\x1b': # space or pure esc (not escape sequences)
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

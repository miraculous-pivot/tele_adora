# dual_hand_teleop.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, termios, tty

# 打印的说明信息
msg = """
Dual-Hand Robotic Arm Teleop
---------------------------
Left Hand Control (Arm Movement & Gripper):
    w/s : +x / -x (forward/backward)
    a/d : +y / -y (left/right)
    q/e : +z / -z (up/down)
    f   : toggle gripper (open/close)

Right Hand Control (Not Implemented):
    (Define your keys and functions here)

space key or 'esc' : force stop and quit
"""

# 左手键位映射
# 格式: '键': (linear.x, linear.y, linear.z)
left_hand_bindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}

# --- 右手键位映射 (待扩展) ---
# 您可以在这里定义右手的键位，例如:
# right_hand_bindings = {
#     'i': (1, 0, 0), # Roll+
#     'k': (-1, 0, 0), # Roll-
#     ...
# }

def get_key(settings):
    """获取单个按键"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_status(arm_vel, gripper_status_str):
    """打印当前状态"""
    return f"Arm Vel: [x:{arm_vel[0]:.2f}, y:{arm_vel[1]:.2f}, z:{arm_vel[2]:.2f}] | Gripper: {gripper_status_str}"

class DualHandTeleopNode(Node):
    def __init__(self):
        super().__init__('dual_hand_teleop_node')
        
        # 创建两个发布者
        self.arm_pub = self.create_publisher(Twist, 'arm_cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Float64, 'gripper_command', 10)
        
        # 速度参数
        self.speed = self.declare_parameter('speed', 0.2).value
        
        # 初始化状态变量
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_linear_z = 0.0
        
        # 夹爪状态: True = Open (1.0), False = Closed (-1.0)
        self.gripper_is_open = True 
        self.last_f_key_state = False # 用于检测'f'键的单次按下

        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Node started. Press keys to control the robot.")
        self.get_logger().info(msg)
        
        self.run()

    def run(self):
        try:
            while rclpy.ok():
                key = get_key(self.settings)

                # --- 左手控制逻辑 ---
                if key in left_hand_bindings:
                    self.target_linear_x = float(left_hand_bindings[key][0])
                    self.target_linear_y = float(left_hand_bindings[key][1])
                    self.target_linear_z = float(left_hand_bindings[key][2])
                elif key == 'f':
                    # 实现'f'键的单次触发切换
                    if not self.last_f_key_state:
                        self.gripper_is_open = not self.gripper_is_open
                    self.last_f_key_state = True
                else:
                    self.target_linear_x = 0.0
                    self.target_linear_y = 0.0
                    self.target_linear_z = 0.0
                    self.last_f_key_state = False

                # --- 右手控制逻辑 (待扩展) ---
                # 在这里添加 if/elif 来处理右手的按键
                # if key in right_hand_bindings:
                #     ...

                # 退出
                if key == ' ' or key == '\x1b': # space or esc
                    break
                
                # 创建并发布消息
                arm_twist = Twist()
                arm_twist.linear.x = self.target_linear_x * self.speed
                arm_twist.linear.y = self.target_linear_y * self.speed
                arm_twist.linear.z = self.target_linear_z * self.speed
                # Angular velocities can be controlled by the right hand
                arm_twist.angular.x = 0.0
                arm_twist.angular.y = 0.0
                arm_twist.angular.z = 0.0
                self.arm_pub.publish(arm_twist)
                
                gripper_cmd = Float64()
                gripper_cmd.data = 1.0 if self.gripper_is_open else -1.0
                self.gripper_pub.publish(gripper_cmd)
                
                # 打印状态
                gripper_str = "OPEN" if self.gripper_is_open else "CLOSED"
                vel_tuple = (arm_twist.linear.x, arm_twist.linear.y, arm_twist.linear.z)
                print(f'\r{print_status(vel_tuple, gripper_str)}', end='')


        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

        finally:
            # 退出前发送停止指令
            self.arm_pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nNode shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = DualHandTeleopNode()
    # 节点内部自己管理循环，所以不需要spin
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

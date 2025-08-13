# teleop_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# 打印的说明信息
msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d

w/s : forward/backward
a/d : turn left/right

arrow keys also work

space key or q : force stop and quit
"""

# 按键与运动的映射
move_bindings = {
    'w': (1, 0, 0, 0),   # 前进
    's': (-1, 0, 0, 0),  # 后退
    'a': (0, 0, 0, 1),   # 左转
    'd': (0, 0, 0, -1),  # 右转
}

# 速度的默认值和增量
speed = .5
turn = .5

def get_key(settings):
    """获取单个按键，处理方向键的转义序列"""
    tty.setraw(sys.stdin.fileno())
    # 使用 select 模块进行非阻塞输入
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        # 处理方向键，它们通常是3个字节的序列: \x1b[A, \x1b[B, ...
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(speed, turn):
    """打印当前速度"""
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"


class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_ubuntu')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.speed = self.declare_parameter('speed', 0.5).value
        self.turn = self.declare_parameter('turn', 0.5).value

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("Node started. Press keys to control the robot.")
        self.get_logger().info(msg)

        self.run()

    def run(self):
        try:
            print(print_vels(self.speed, self.turn))
            while rclpy.ok():
                key = get_key(self.settings)

                # 基础按键
                if key in move_bindings:
                    self.x = float(move_bindings[key][0])
                    self.th = float(move_bindings[key][3])
                # 方向键
                elif key == '\x1b[A': # Up
                    self.x = 1.0
                    self.th = 0.0
                elif key == '\x1b[B': # Down
                    self.x = -1.0
                    self.th = 0.0
                elif key == '\x1b[C': # Right
                    self.x = 0.0
                    self.th = -1.0
                elif key == '\x1b[D': # Left
                    self.x = 0.0
                    self.th = 1.0
                # 停止并退出
                elif key == ' ' or key == 'q':
                    break
                else:
                    # 如果没有按键，则机器人停止
                    self.x = 0.0
                    self.th = 0.0

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn
                self.publisher_.publish(twist)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

        finally:
            # 确保在退出前发送停止命令
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    # 我们自己管理循环，所以不需要 rclpy.spin(node)
    # 当run()方法结束时，节点生命周期也应该结束
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

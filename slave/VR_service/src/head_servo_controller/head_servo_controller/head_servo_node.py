import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from unity_robotics_demo_msgs.msg import PosRot
from math import atan2, asin

class HeadServoController(Node):

    def __init__(self):
        super().__init__('head_servo_controller')
        
        # 创建发布者，发布到 "adora_robot/head/servo_cmd"
        self.publisher_ = self.create_publisher(Float32MultiArray, 'adora_robot/head/servo_cmd', 10)
        
        # 创建订阅者，订阅 "/main_camera_transform"
        self.subscription = self.create_subscription(
            PosRot,
            '/camera_transform',
            self.transform_callback,
            10)
            
        self.get_logger().info('Head Servo Controller Node has been started.')

    def transform_callback(self, msg):
        """
        处理 /camera_transform 话题数据的回调函数。
        """
        # 从四元数中提取俯仰角 (pitch) 和旋转角 (yaw)
        # 这里使用了一个简化的四元数到欧拉角的转换方法。
        # 注意：这是一种简化的方法，可能不适用于所有情况，特别是 Gimbal Lock。
        
        # 从四元数中提取俯仰角 (pitch)
        # 俯仰角与 y 轴旋转相关
        # 头部向上偏移（仰视）为正，对应 pitch 角度增加
        # 头部向下偏移（俯视）为负，对应 pitch 角度减小
        
        # yaw (rotation about Z), pitch (rotation about Y), roll (rotation about X)
        
        # t0 = +2.0 * (msg.rot_w * msg.rot_x + msg.rot_y * msg.rot_z)
        # t1 = +1.0 - 2.0 * (msg.rot_x * msg.rot_x + msg.rot_y * msg.rot_y)
        # roll = atan2(t0, t1)

        t2 = +2.0 * (msg.rot_w * msg.rot_y - msg.rot_z * msg.rot_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_angle = asin(t2)

        t3 = +2.0 * (msg.rot_w * msg.rot_z + msg.rot_x * msg.rot_y)
        t4 = +1.0 - 2.0 * (msg.rot_y * msg.rot_y + msg.rot_z * msg.rot_z)
        yaw_angle = atan2(t3, t4)
        
        # 调整角度符号以匹配云台控制方式
        # 俯视（向下）为负 -> pitch 负
        # 仰视（向上）为正 -> pitch 正
        # 逆时针（向左看）为正 -> yaw 正
        # 顺时针（向右看）为负 -> yaw 负
        # 这里的符号可能需要根据你的具体相机和坐标系设置进行微调
        # 这里我们假设四元数转换出来的角度与云台的控制方向一致。
        
        # 创建并发布 Float32MultiArray 消息
        servo_cmd_msg = Float32MultiArray()
        servo_cmd_msg.data = [pitch_angle, yaw_angle]
        
        self.publisher_.publish(servo_cmd_msg)
        self.get_logger().info(f'Published servo command: [{pitch_angle:.2f}, {yaw_angle:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    node = HeadServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
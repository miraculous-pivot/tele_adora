import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from piper_sdk import C_PiperInterface_V2
import time
import threading

# 注意：你需要根据你的实际硬件配置修改CAN端口和机械臂的参数
LEFT_ARM_CAN = "can0"
RIGHT_ARM_CAN = "can1"

# 坐标转换和缩放因子
# ROS中的单位是米和弧度，Piper SDK需要的是整数
# 你需要根据实际情况调整这些缩放比例
# POS_SCALE_FACTOR = 10000000  # 1m -> 1000 units
# ROT_SCALE_FACTOR = 1000000 # 1rad -> 1000 units
POS_SCALE_FACTOR = 20000  # 1m -> 1000 units
ROT_SCALE_FACTOR = 10000 # 1rad -> 1000 units

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        self.get_logger().info("Arm Controller Node started.")

        # --- 初始化左右臂 ---
        self.left_arm = self.initialize_arm(LEFT_ARM_CAN, "Left")
        self.right_arm = self.initialize_arm(RIGHT_ARM_CAN, "Right")
        self.left_arm_pose = {
                'joint_1': int(0 * POS_SCALE_FACTOR),
                'joint_2': int(0 * POS_SCALE_FACTOR),
                'joint_3': int(0 * POS_SCALE_FACTOR),
                'joint_4': int(0 * ROT_SCALE_FACTOR),
                'joint_5': int(0 * ROT_SCALE_FACTOR),
                'joint_6': int(0 * ROT_SCALE_FACTOR),
            }
        self.right_arm_pose = {
                'joint_1': int(0 * POS_SCALE_FACTOR),
                'joint_2': int(0 * POS_SCALE_FACTOR),
                'joint_3': int(0 * POS_SCALE_FACTOR),
                'joint_4': int(0 * ROT_SCALE_FACTOR),
                'joint_5': int(0 * ROT_SCALE_FACTOR),
                'joint_6': int(0 * ROT_SCALE_FACTOR),
            }
        # --- 创建订阅者 ---
        if self.left_arm:
            self.left_arm_sub = self.create_subscription(
                Pose, '/left_arm/pose_cmd', self.left_arm_callback, 10)
            self.left_gripper_sub = self.create_subscription(
                Bool, '/left_arm/gripper_cmd', self.left_gripper_callback, 10)
            self.get_logger().info("left arm topic sub successful")

        if self.right_arm:
            self.right_arm_sub = self.create_subscription(
                Pose, '/right_arm/pose_cmd', self.right_arm_callback, 10)
            self.right_gripper_sub = self.create_subscription(
                Bool, '/right_arm/gripper_cmd', self.right_gripper_callback, 10)
            self.get_logger().info("right arm topic sub successful")
            
            
        
        # 创建一个锁以避免在回调中同时访问一个机械臂
        self.left_lock = threading.Lock()
        self.right_lock = threading.Lock()
        
        self.get_logger().info("Node initialization complete. Waiting for commands.")

    def initialize_arm(self, can_name, arm_id):
        """初始化单个机械臂"""
        self.get_logger().info(f"Initializing {arm_id} Arm on {can_name}...")
        try:
            arm = C_PiperInterface_V2(can_name=can_name, judge_flag=False)
            arm.ConnectPort()
            
            start_time = time.time()
            
            while not arm.EnablePiper():
                time.sleep(0.01)
                if time.time() - start_time > 5.0: # 5秒超时
                    raise RuntimeError(f"Failed to enable {arm_id} Arm: Timeout")
            
            # 设置为末端位姿控制模式
            arm.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.get_logger().info(f"{arm_id} Arm Initialized Successfully.")
            return arm
        except Exception as e:
            self.get_logger().error(f"Failed to initialize {arm_id} Arm: {e}")
            return None

    def get_current_pose(self, arm: C_PiperInterface_V2):
        """从Piper SDK获取当前末端位姿"""
        if not arm:
            return None
        for _ in range(3):
            pose_msg = arm.GetArmEndPoseMsgs()
            # print(pose_msg)
            # 检查消息对象和end_pose属性
            if pose_msg and hasattr(pose_msg, 'end_pose') and pose_msg.end_pose:
                end_pose_obj = pose_msg.end_pose

                # 检查end_pose_obj是否包含X_axis等属性
                if hasattr(end_pose_obj, 'X_axis'):
                    # print("yes")
                    return {
                        'X': end_pose_obj.X_axis,
                        'Y': end_pose_obj.Y_axis,
                        'Z': end_pose_obj.Z_axis,
                        'RX': end_pose_obj.RX_axis,
                        'RY': end_pose_obj.RY_axis,
                        'RZ': end_pose_obj.RZ_axis,
                        'timestamp': pose_msg.time_stamp,  # 从顶级对象获取
                        'Hz': pose_msg.Hz                  # 从顶级对象获取
                    }
            time.sleep(0.01)
        return None
    def get_current_joint_angles(self, arm: C_PiperInterface_V2):
        """从Piper SDK获取当前关节角"""
        if not arm:
            return None
        for _ in range(3):
            joint_msg = arm.GetArmJointMsgs()
            if joint_msg and hasattr(joint_msg, 'joint_state') and joint_msg.joint_state:
                joint_ctrl_obj = joint_msg.joint_state
                if hasattr(joint_ctrl_obj, 'joint_1'):
                    return {
                        'joint_1': joint_ctrl_obj.joint_1,
                        'joint_2': joint_ctrl_obj.joint_2,
                        'joint_3': joint_ctrl_obj.joint_3,
                        'joint_4': joint_ctrl_obj.joint_4,
                        'joint_5': joint_ctrl_obj.joint_5,
                        'joint_6': joint_ctrl_obj.joint_6,
                        'timestamp': joint_msg.time_stamp,
                        'Hz': joint_msg.Hz
                    }
            time.sleep(0.01)
        return None

    def left_arm_callback(self, msg: Pose):
        if not self.left_arm:
            return
        
        with self.left_lock:
            # 在每次控制前获取最新的位姿
            # current_pose = self.get_current_pose(self.left_arm)
            current_joint = self.get_current_joint_angles(self.left_arm)
            # 如果无法获取当前位姿，则放弃本次控制
            # if not current_pose:
            #     self.get_logger().warn("Left arm callback skipped: Could not get current pose.")
            #     return

            # 计算目标位姿
            # target_pose = {
            #     'X': current_pose['X'] + int(msg.position.x * POS_SCALE_FACTOR),
            #     'Y': current_pose['Y'] + int(msg.position.y * POS_SCALE_FACTOR),
            #     'Z': current_pose['Z'] + int(msg.position.z * POS_SCALE_FACTOR),
            #     'RX': current_pose['RX'] + int(msg.orientation.x * ROT_SCALE_FACTOR),
            #     'RY': current_pose['RY'] + int(msg.orientation.y * ROT_SCALE_FACTOR),
            #     'RZ': current_pose['RZ'] + int(msg.orientation.z * ROT_SCALE_FACTOR),
            # }
            self.left_arm_pose = {
                'joint_1': current_joint['joint_1'] + int(msg.position.x * 57295.7795),
                'joint_2': current_joint['joint_2'] + int(msg.position.y * 57295.7795),
                'joint_3': current_joint['joint_3'] + int(msg.position.z * 57295.7795),
                'joint_4': current_joint['joint_4'] + int(msg.orientation.x * 57295.7795),
                'joint_5': current_joint['joint_5'] + int(msg.orientation.y * 57295.7795),
                'joint_6': current_joint['joint_6'] + int(msg.orientation.z * 57295.7795),
            }
            print(self.left_arm_pose)
            print("1")
            # 发送控制指令
            try:
                #self.left_arm.EndPoseCtrl(**target_pose)
                self.left_arm.JointCtrl(**self.left_arm_pose)
            except ValueError as e:
                print(f"Control failed: {str(e)}")

    def left_gripper_callback(self, msg: Bool):
        if not self.left_arm:
            return
        with self.left_lock:
            is_open = msg.data
            # print("left")
            # print(is_open)
            if is_open:
                self.left_arm.GripperCtrl(gripper_angle=100000,gripper_effort=200,gripper_code=1)
            else:
                self.left_arm.GripperCtrl(gripper_angle=0,gripper_effort=500,gripper_code=1)
                

    def right_arm_callback(self, msg: Pose):
        if not self.right_arm:
            return
            
        with self.right_lock:
            # 在每次控制前获取最新的位姿
            # current_pose = self.get_current_pose(self.right_arm)
            current_joint = self.get_current_joint_angles(self.right_arm)
            # 如果无法获取当前位姿，则放弃本次控制
            # if not current_pose:
            #     self.get_logger().warn("Right arm callback skipped: Could not get current pose.")
            #     return

            # 计算目标位姿
            # target_pose = {
            #     'X': current_pose['X'] + int(msg.position.x * POS_SCALE_FACTOR),
            #     'Y': current_pose['Y'] + int(msg.position.y * POS_SCALE_FACTOR),
            #     'Z': current_pose['Z'] + int(msg.position.z * POS_SCALE_FACTOR),
            #     'RX': current_pose['RX'] + int(msg.orientation.x * ROT_SCALE_FACTOR),
            #     'RY': current_pose['RY'] + int(msg.orientation.y * ROT_SCALE_FACTOR),
            #     'RZ': current_pose['RZ'] + int(msg.orientation.z * ROT_SCALE_FACTOR),
            # }
            self.right_arm_pose = {
                'joint_1': current_joint['joint_1'] + int(msg.position.x * 57295.7795),
                'joint_2': current_joint['joint_2'] + int(msg.position.y * 57295.7795),
                'joint_3': current_joint['joint_3'] + int(msg.position.z * 57295.7795),
                'joint_4': current_joint['joint_4'] + int(msg.orientation.x * 57295.7795),
                'joint_5': current_joint['joint_5'] + int(msg.orientation.y * 57295.7795),
                'joint_6': current_joint['joint_6'] + int(msg.orientation.z * 57295.7795),
            }
            print(self.right_arm_pose)
            print("2")
            # 发送控制指令
            # self.right_arm.EndPoseCtrl(**target_pose)
            self.right_arm.JointCtrl(**self.right_arm_pose)

    def right_gripper_callback(self, msg: Bool):
        if not self.right_arm:
            return
        with self.right_lock:
            is_open = msg.data
            # print("right")
            # print(is_open)
            if is_open:
                self.right_arm.GripperCtrl(gripper_angle=100000,gripper_effort=200,gripper_code=1)
            else:
                self.right_arm.GripperCtrl(gripper_angle=0,gripper_effort=500,gripper_code=1)
                
    def shutdown(self):
        """安全关闭节点和机械臂"""
        self.get_logger().info("Shutting down...")
        if self.left_arm:
            self.left_arm.DisableArm()
            self.left_arm.DisconnectPort()
        if self.right_arm:
            self.right_arm.DisableArm()
            self.right_arm.DisconnectPort()

def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
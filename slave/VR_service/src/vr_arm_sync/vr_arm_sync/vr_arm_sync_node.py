#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unity_robotics_demo_msgs.msg import PosRot
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool, UInt32
import threading
import time
import sys
import os

# 添加piper_sdk到Python路径 - 使用相对路径
script_dir = os.path.dirname(os.path.abspath(__file__))
tele_adora_root = os.path.join(script_dir, '..', '..', '..', '..', '..')
piper_sdk_path = os.path.join(tele_adora_root, 'piper_sdk')
sys.path.append(piper_sdk_path)
from piper_sdk import C_PiperInterface_V2

# 注意：你需要根据你的实际硬件配置修改CAN端口和机械臂的参数
LEFT_ARM_CAN = "can0"
RIGHT_ARM_CAN = "can1"

# 坐标转换和缩放因子
# ROS中的单位是米和弧度，Piper SDK需要的是整数
# 你需要根据实际情况调整这些缩放比例
POS_SCALE_FACTOR = 10000000  # 1m -> 1000 units
ROT_SCALE_FACTOR = 1000000   # 1rad -> 1000 units


class VRArmSyncNode(Node):
    def __init__(self):
        super().__init__('vr_arm_sync_node')
        
        # 初始化变量
        self.left_calibration_trigger = False
        self.right_calibration_trigger = False
        self.left_arm_pose = None
        self.right_arm_pose = None
        self.left_vr_base_pose = None
        self.right_vr_base_pose = None
        self.is_left_calibrated = False
        self.is_right_calibrated = False
        
        # Gripper状态和时间控制
        self.left_gripper_state = False  # False: 关闭, True: 打开
        self.right_gripper_state = False
        self.left_last_gripper_time = 0.0
        self.right_last_gripper_time = 0.0
        self.gripper_interval = 2.0  # 2秒间隔
        
        # 升降电机控制变量 - 注意：pos_x和pos_y现在用于底盘控制
        self.current_lifting_position = 0  # 当前升降高度 (mm)
        self.max_lifting_distance = 700    # 最大升降距离 (mm) - 根据实际机器人配置
        self.lifting_step = 50             # 每次升降步长 (mm)
        # 升降控制现在可以使用其他按钮，比如rot_w字段或其他未使用的字段
        self.lifting_interval = 1.0        # 升降控制间隔 (秒)
        
        # 底盘控制变量
        self.chassis_linear_scale = 1.0    # 线速度缩放系数 (m/s)
        self.chassis_angular_scale = 1.0   # 角速度缩放系数 (rad/s)
        self.chassis_max_linear = 1.0      # 最大线速度 (m/s)
        self.chassis_max_angular = 1.0     # 最大角速度 (rad/s)
        
        # 线程锁
        self.left_lock = threading.Lock()
        self.right_lock = threading.Lock()
        
        # --- 初始化左右臂 ---
        self.get_logger().info("Initializing arms...")
        self.left_arm = self.initialize_arm(LEFT_ARM_CAN, "Left")
        self.right_arm = self.initialize_arm(RIGHT_ARM_CAN, "Right")
        
        # 打印初始化结果
        if self.left_arm:
            self.get_logger().info("Left arm initialized successfully")
        else:
            self.get_logger().error("Left arm initialization failed")
            
        if self.right_arm:
            self.get_logger().info("Right arm initialized successfully")
        else:
            self.get_logger().error("Right arm initialization failed")
        
        # 订阅VR控制器输入话题
        self.left_buttons_sub = self.create_subscription(
            PosRot,
            '/left_buttons',
            self.left_buttons_callback,
            10
        )
        
        self.left_analog_sub = self.create_subscription(
            PosRot,
            '/left_analog',
            self.left_analog_callback,
            10
        )
        
        self.right_analog_sub = self.create_subscription(
            PosRot,
            '/right_buttons',
            self.right_analog_callback,
            10
        )
        
        # 订阅VR控制器位姿话题
        self.left_transform_sub = self.create_subscription(
            PosRot,
            '/left_transform',
            self.left_transform_callback,
            10
        )
        
        self.right_transform_sub = self.create_subscription(
            PosRot,
            '/right_transform',
            self.right_transform_callback,
            10
        )
        
        # 发布机械臂当前位姿话题（用于外部监控）
        self.left_arm_pose_pub = self.create_publisher(
            Pose,
            '/left_arm/pose_cmd',
            10
        )
        
        self.right_arm_pose_pub = self.create_publisher(
            Pose,
            '/right_arm/pose_cmd',
            10
        )
        
        # 发布gripper状态
        self.left_gripper_pub = self.create_publisher(
            Bool,
            '/left_arm/gripper_cmd',
            10
        )
        
        self.right_gripper_pub = self.create_publisher(
            Bool,
            '/right_arm/gripper_cmd', 
            10
        )
        
        # 发布升降电机控制指令
        self.lifting_motor_pub = self.create_publisher(
            UInt32,
            '/adora/lifting_motor/cmd',
            10
        )
        
        # 订阅升降电机状态
        self.lifting_motor_sub = self.create_subscription(
            UInt32,
            '/adora/lifting_motor/states',
            self.lifting_motor_states_callback,
            10
        )
        
        # 发布底盘速度控制指令
        self.chassis_velocity_pub = self.create_publisher(
            Twist,
            '/dt/velocity_ctrl',
            10
        )
        
        # 创建定时器定期发布机械臂位姿
        self.pose_timer = self.create_timer(0.1, self.publish_arm_poses)  # 10Hz
        
        self.get_logger().info('VR Arm Sync Node initialized')

    def initialize_arm(self, can_name, arm_id):
        """初始化单个机械臂"""
        self.get_logger().info(f"Initializing {arm_id} Arm on {can_name}...")
        try:
            arm = C_PiperInterface_V2(can_name=can_name, judge_flag=False)
            arm.ConnectPort()
            
            start_time = time.time()
            
            while not arm.EnablePiper():
                time.sleep(0.01)
                if time.time() - start_time > 5.0:  # 5秒超时
                    raise RuntimeError(f"Failed to enable {arm_id} Arm: Timeout")
            
            # 设置为末端位姿控制模式
            arm.MotionCtrl_2(ctrl_mode=0x01, move_mode=0x00)
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
            # 检查消息对象和end_pose属性
            if pose_msg and hasattr(pose_msg, 'end_pose') and pose_msg.end_pose:
                end_pose_obj = pose_msg.end_pose

                # 检查end_pose_obj是否包含X_axis等属性
                if hasattr(end_pose_obj, 'X_axis'):
                    return {
                        'X': end_pose_obj.X_axis,
                        'Y': end_pose_obj.Y_axis,
                        'Z': end_pose_obj.Z_axis,
                        'RX': end_pose_obj.RX_axis,
                        'RY': end_pose_obj.RY_axis,
                        'RZ': end_pose_obj.RZ_axis,
                        'timestamp': pose_msg.time_stamp,
                        'Hz': pose_msg.Hz
                    }
            time.sleep(0.01)
        return None

    def publish_arm_poses(self):
        """定期发布机械臂当前位姿"""
        # 发布左臂位姿
        if self.left_arm:
            current_pose = self.get_current_pose(self.left_arm)
            if current_pose:
                self.left_arm_pose = current_pose
                pose_msg = Pose()
                pose_msg.position.x = current_pose['X'] / POS_SCALE_FACTOR
                pose_msg.position.y = current_pose['Y'] / POS_SCALE_FACTOR
                pose_msg.position.z = current_pose['Z'] / POS_SCALE_FACTOR
                pose_msg.orientation.x = current_pose['RX'] / ROT_SCALE_FACTOR
                pose_msg.orientation.y = current_pose['RY'] / ROT_SCALE_FACTOR
                pose_msg.orientation.z = current_pose['RZ'] / ROT_SCALE_FACTOR
                pose_msg.orientation.w = 1.0  # 这里简化处理
                self.left_arm_pose_pub.publish(pose_msg)
        
        # 发布右臂位姿
        if self.right_arm:
            current_pose = self.get_current_pose(self.right_arm)
            if current_pose:
                self.right_arm_pose = current_pose
                pose_msg = Pose()
                pose_msg.position.x = current_pose['X'] / POS_SCALE_FACTOR
                pose_msg.position.y = current_pose['Y'] / POS_SCALE_FACTOR
                pose_msg.position.z = current_pose['Z'] / POS_SCALE_FACTOR
                pose_msg.orientation.x = current_pose['RX'] / ROT_SCALE_FACTOR
                pose_msg.orientation.y = current_pose['RY'] / ROT_SCALE_FACTOR
                pose_msg.orientation.z = current_pose['RZ'] / ROT_SCALE_FACTOR
                pose_msg.orientation.w = 1.0  # 这里简化处理
                self.right_arm_pose_pub.publish(pose_msg)
    
    def left_buttons_callback(self, msg):
        """处理左手控制器按钮输入"""
        with self.left_lock:
            # 检查rot_z是否为1（校准触发信号）- float32类型需要容差检测
            if abs(msg.rot_z - 1.0) < 0.001:
                if not self.left_calibration_trigger:
                    self.left_calibration_trigger = True
                    self.get_logger().info('Left controller calibration triggered')
                    self._calibrate_left_arm()
            else:
                self.left_calibration_trigger = False
            
            # 检查rot_y是否为1（gripper控制信号）- float32类型需要容差检测
            self.get_logger().info(f'Left buttons rot_y value: {msg.rot_y}')
            if abs(msg.rot_y - 1.0) < 0.001:
                self.get_logger().info('Left gripper trigger detected!')
                current_time = time.time()
                time_since_last = current_time - self.left_last_gripper_time
                self.get_logger().info(f'Time since last gripper action: {time_since_last:.2f}s')
                
                if time_since_last >= self.gripper_interval:
                    self.left_last_gripper_time = current_time
                    self.left_gripper_state = not self.left_gripper_state
                    self.get_logger().info(f'Left gripper state changing to: {"opened" if self.left_gripper_state else "closed"}')
                    self._control_left_gripper(self.left_gripper_state)
                    self.get_logger().info(f'Left gripper {"opened" if self.left_gripper_state else "closed"}')
                else:
                    self.get_logger().warn(f'Left gripper action blocked - need to wait {self.gripper_interval - time_since_last:.2f}s more')
            elif msg.rot_y != 0.0:
                self.get_logger().debug(f'Left buttons rot_y is {msg.rot_y}, not 1.0')
            
            # 升降控制 - pos_x和pos_y为按钮输入（值为1时触发）
            # pos_x = 1: 上升控制
            # pos_y = 1: 下降控制
            if abs(msg.pos_x - 1.0) < 0.001:
                self.get_logger().info('Left lifting UP trigger detected!')
                current_time = time.time()
                if not hasattr(self, 'left_last_pos_x_time'):
                    self.left_last_pos_x_time = 0.0
                time_since_last = current_time - self.left_last_pos_x_time
                self.get_logger().info(f'Time since last lifting UP action: {time_since_last:.2f}s')
                
                if time_since_last >= self.lifting_interval:
                    self.left_last_pos_x_time = current_time
                    new_position = min(self.current_lifting_position + self.lifting_step, self.max_lifting_distance)
                    self.get_logger().info(f'Lifting UP: {self.current_lifting_position}mm -> {new_position}mm')
                    self._control_lifting_motor(new_position)
                else:
                    self.get_logger().warn(f'Lifting UP action blocked - need to wait {self.lifting_interval - time_since_last:.2f}s more')
            elif msg.pos_x != 0.0:
                self.get_logger().debug(f'Left buttons pos_x is {msg.pos_x}, not 1.0')
            
            if abs(msg.pos_y - 1.0) < 0.001:
                self.get_logger().info('Left lifting DOWN trigger detected!')
                current_time = time.time()
                if not hasattr(self, 'left_last_pos_y_time'):
                    self.left_last_pos_y_time = 0.0
                time_since_last = current_time - self.left_last_pos_y_time
                self.get_logger().info(f'Time since last lifting DOWN action: {time_since_last:.2f}s')
                
                if time_since_last >= self.lifting_interval:
                    self.left_last_pos_y_time = current_time
                    new_position = max(self.current_lifting_position - self.lifting_step, 0)
                    self.get_logger().info(f'Lifting DOWN: {self.current_lifting_position}mm -> {new_position}mm')
                    self._control_lifting_motor(new_position)
                else:
                    self.get_logger().warn(f'Lifting DOWN action blocked - need to wait {self.lifting_interval - time_since_last:.2f}s more')
            elif msg.pos_y != 0.0:
                self.get_logger().debug(f'Left buttons pos_y is {msg.pos_y}, not 1.0')
    
    def left_analog_callback(self, msg):
        """处理左手控制器模拟摇杆输入"""
        with self.left_lock:
            # 底盘控制 - pos_x和pos_y为摇杆模拟量输入
            # pos_x: 摇杆横轴，向右为正 -> 角速度控制 (左转为负，右转为正)
            # pos_y: 摇杆纵轴，向前为正 -> 线速度控制 (前进为正，后退为负)
            if abs(msg.pos_x) > 0.1 or abs(msg.pos_y) > 0.1:  # 设置死区，避免微小抖动
                # 计算目标速度
                linear_velocity = msg.pos_y * self.chassis_linear_scale
                angular_velocity = -msg.pos_x * self.chassis_angular_scale  # 左转为正，所以取负号
                
                # 限制最大速度
                linear_velocity = max(-self.chassis_max_linear, min(linear_velocity, self.chassis_max_linear))
                angular_velocity = max(-self.chassis_max_angular, min(angular_velocity, self.chassis_max_angular))
                
                self.get_logger().debug(f'Chassis control: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}')
                self._control_chassis_velocity(linear_velocity, angular_velocity)
            else:
                # 摇杆回中，停止底盘运动
                if abs(msg.pos_x) < 0.05 and abs(msg.pos_y) < 0.05:  # 更小的死区用于停止
                    self._control_chassis_velocity(0.0, 0.0)
    
    def right_analog_callback(self, msg):
        """处理右手控制器模拟量输入"""
        with self.right_lock:
            # 检查rot_z是否为1（校准触发信号）- float32类型需要容差检测
            if abs(msg.rot_z - 1.0) < 0.001:
                if not self.right_calibration_trigger:
                    self.right_calibration_trigger = True
                    self.get_logger().info('Right controller calibration triggered')
                    self._calibrate_right_arm()
            else:
                self.right_calibration_trigger = False
            
            # 检查rot_y是否为1（gripper控制信号）- float32类型需要容差检测
            self.get_logger().debug(f'Right analog rot_y value: {msg.rot_y}')
            if abs(msg.rot_y - 1.0) < 0.001:
                self.get_logger().info('Right gripper trigger detected!')
                current_time = time.time()
                time_since_last = current_time - self.right_last_gripper_time
                self.get_logger().info(f'Time since last gripper action: {time_since_last:.2f}s')
                
                if time_since_last >= self.gripper_interval:
                    self.right_last_gripper_time = current_time
                    self.right_gripper_state = not self.right_gripper_state
                    self.get_logger().info(f'Right gripper state changing to: {"opened" if self.right_gripper_state else "closed"}')
                    self._control_right_gripper(self.right_gripper_state)
                    self.get_logger().info(f'Right gripper {"opened" if self.right_gripper_state else "closed"}')
                else:
                    self.get_logger().warn(f'Right gripper action blocked - need to wait {self.gripper_interval - time_since_last:.2f}s more')
            elif msg.rot_y != 0.0:
                self.get_logger().debug(f'Right analog rot_y is {msg.rot_y}, not 1.0')
    
    def left_transform_callback(self, msg):
        """处理左手控制器位姿变化"""
        # 如果刚刚校准完成，将当前位姿设为基准
        if self.is_left_calibrated and self.left_vr_base_pose is None:
            self.left_vr_base_pose = msg
            self.get_logger().info('Left VR base pose set')
            return
        
        if self.is_left_calibrated and self.left_vr_base_pose is not None and self.left_arm:
            with self.left_lock:
                # 计算VR控制器相对于基准位姿的变化
                delta_pos = [
                    msg.pos_x - self.left_vr_base_pose.pos_x,
                    msg.pos_y - self.left_vr_base_pose.pos_y,
                    msg.pos_z - self.left_vr_base_pose.pos_z
                ]
                
                delta_rot = [
                    msg.rot_x - self.left_vr_base_pose.rot_x,
                    msg.rot_y - self.left_vr_base_pose.rot_y,
                    msg.rot_z - self.left_vr_base_pose.rot_z,
                ]
                
                # 获取当前机械臂位姿
                current_pose = self.get_current_pose(self.left_arm)
                if current_pose:
                    # 计算目标位姿
                    target_pose = {
                        'X': current_pose['X'] + int(delta_pos[0] * POS_SCALE_FACTOR),
                        'Y': current_pose['Y'] + int(delta_pos[1] * POS_SCALE_FACTOR),
                        'Z': current_pose['Z'] + int(delta_pos[2] * POS_SCALE_FACTOR),
                        'RX': current_pose['RX'] + int(delta_rot[0] * ROT_SCALE_FACTOR),
                        'RY': current_pose['RY'] + int(delta_rot[1] * ROT_SCALE_FACTOR),
                        'RZ': current_pose['RZ'] + int(delta_rot[2] * ROT_SCALE_FACTOR),
                    }
                    
                    # 发送控制指令
                    try:
                        self.left_arm.EndPoseCtrl(**target_pose)
                    except ValueError as e:
                        self.get_logger().warn(f"Left arm control failed: {str(e)}")
    
    def right_transform_callback(self, msg):
        """处理右手控制器位姿变化"""
        # 如果刚刚校准完成，将当前位姿设为基准
        if self.is_right_calibrated and self.right_vr_base_pose is None:
            self.right_vr_base_pose = msg
            self.get_logger().info('Right VR base pose set')
            return
        
        if self.is_right_calibrated and self.right_vr_base_pose is not None and self.right_arm:
            with self.right_lock:
                # 计算VR控制器相对于基准位姿的变化
                delta_pos = [
                    msg.pos_x - self.right_vr_base_pose.pos_x,
                    msg.pos_y - self.right_vr_base_pose.pos_y,
                    msg.pos_z - self.right_vr_base_pose.pos_z
                ]
                
                delta_rot = [
                    msg.rot_x - self.right_vr_base_pose.rot_x,
                    msg.rot_y - self.right_vr_base_pose.rot_y,
                    msg.rot_z - self.right_vr_base_pose.rot_z,
                ]
                
                # 获取当前机械臂位姿
                current_pose = self.get_current_pose(self.right_arm)
                if current_pose:
                    # 计算目标位姿
                    target_pose = {
                        'X': current_pose['X'] + int(delta_pos[0] * POS_SCALE_FACTOR),
                        'Y': current_pose['Y'] + int(delta_pos[1] * POS_SCALE_FACTOR),
                        'Z': current_pose['Z'] + int(delta_pos[2] * POS_SCALE_FACTOR),
                        'RX': current_pose['RX'] + int(delta_rot[0] * ROT_SCALE_FACTOR),
                        'RY': current_pose['RY'] + int(delta_rot[1] * ROT_SCALE_FACTOR),
                        'RZ': current_pose['RZ'] + int(delta_rot[2] * ROT_SCALE_FACTOR),
                    }
                    
                    # 发送控制指令
                    try:
                        self.right_arm.EndPoseCtrl(**target_pose)
                    except ValueError as e:
                        self.get_logger().warn(f"Right arm control failed: {str(e)}")
    
    def _control_left_gripper(self, is_open):
        """控制左机械臂gripper"""
        self.get_logger().info(f'_control_left_gripper called with is_open={is_open}')
        
        if not self.left_arm:
            self.get_logger().error('Left arm not initialized - cannot control gripper')
            return
        
        with self.left_lock:
            try:
                if is_open:
                    self.get_logger().info('Sending left gripper OPEN command')
                    self.left_arm.GripperCtrl(gripper_angle=100000, gripper_effort=200, gripper_code=1)
                else:
                    self.get_logger().info('Sending left gripper CLOSE command')
                    self.left_arm.GripperCtrl(gripper_angle=0, gripper_effort=500, gripper_code=1)
                
                self.get_logger().info('Left gripper command sent successfully')
                
                # 发布gripper状态
                gripper_msg = Bool()
                gripper_msg.data = is_open
                self.left_gripper_pub.publish(gripper_msg)
                self.get_logger().info(f'Left gripper state published: {is_open}')
                
            except Exception as e:
                self.get_logger().error(f'Left gripper control failed: {str(e)}')
    
    def _control_right_gripper(self, is_open):
        """控制右机械臂gripper"""
        self.get_logger().info(f'_control_right_gripper called with is_open={is_open}')
        
        if not self.right_arm:
            self.get_logger().error('Right arm not initialized - cannot control gripper')
            return
        
        with self.right_lock:
            try:
                if is_open:
                    self.get_logger().info('Sending right gripper OPEN command')
                    self.right_arm.GripperCtrl(gripper_angle=100000, gripper_effort=200, gripper_code=1)
                else:
                    self.get_logger().info('Sending right gripper CLOSE command')
                    self.right_arm.GripperCtrl(gripper_angle=0, gripper_effort=500, gripper_code=1)
                
                self.get_logger().info('Right gripper command sent successfully')
                
                # 发布gripper状态
                gripper_msg = Bool()
                gripper_msg.data = is_open
                self.right_gripper_pub.publish(gripper_msg)
                self.get_logger().info(f'Right gripper state published: {is_open}')
                
            except Exception as e:
                self.get_logger().error(f'Right gripper control failed: {str(e)}')
    
    def _control_lifting_motor(self, target_position):
        """控制升降电机到指定位置"""
        self.get_logger().info(f'_control_lifting_motor called with target_position={target_position}mm')
        
        try:
            # 确保目标位置在有效范围内
            target_position = max(0, min(target_position, self.max_lifting_distance))
            
            # 发布升降电机控制指令
            lifting_msg = UInt32()
            lifting_msg.data = int(target_position)
            self.lifting_motor_pub.publish(lifting_msg)
            
            self.get_logger().info(f'Lifting motor command sent: {target_position}mm')
            
        except Exception as e:
            self.get_logger().error(f'Lifting motor control failed: {str(e)}')
    
    def lifting_motor_states_callback(self, msg):
        """升降电机状态回调函数"""
        self.current_lifting_position = int(msg.data)
        self.get_logger().debug(f'Current lifting position updated: {self.current_lifting_position}mm')
    
    def _control_chassis_velocity(self, linear_velocity, angular_velocity):
        """控制底盘移动速度"""
        try:
            # 创建Twist消息
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = angular_velocity
            
            # 发布速度指令
            self.chassis_velocity_pub.publish(twist_msg)
            
            if linear_velocity != 0.0 or angular_velocity != 0.0:
                self.get_logger().debug(f'Chassis velocity command sent: linear={linear_velocity:.2f}m/s, angular={angular_velocity:.2f}rad/s')
            
        except Exception as e:
            self.get_logger().error(f'Chassis velocity control failed: {str(e)}')
    
    def _calibrate_left_arm(self):
        """校准左机械臂"""
        if self.left_arm:
            # 重置基准位姿，下一个transform消息将被用作基准
            self.left_vr_base_pose = None
            self.is_left_calibrated = True
            self.get_logger().info('Left arm calibration initiated - waiting for next transform message')
        else:
            self.get_logger().warn('Cannot calibrate left arm: arm not initialized')
    
    def _calibrate_right_arm(self):
        """校准右机械臂"""
        if self.right_arm:
            # 重置基准位姿，下一个transform消息将被用作基准
            self.right_vr_base_pose = None
            self.is_right_calibrated = True
            self.get_logger().info('Right arm calibration initiated - waiting for next transform message')
        else:
            self.get_logger().warn('Cannot calibrate right arm: arm not initialized')
    
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
    
    node = VRArmSyncNode()
    
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

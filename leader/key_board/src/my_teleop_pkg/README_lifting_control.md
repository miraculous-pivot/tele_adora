# 双臂机器人+升降电机键盘控制

这个包提供了同时控制双臂机器人和升降电机的键盘控制功能。

## 功能特性

- 双臂6自由度控制
- 双臂夹爪控制
- 升降电机位置控制
- 实时状态反馈

## 按键映射

### 左臂控制
- `w/s`: X轴 前进/后退
- `a/d`: Y轴 左/右
- `q/e`: Z轴 上/下
- `r/f`: 滚转角 +/-
- `t/g`: 俯仰角 +/-
- `y/h`: 偏航角 +/-
- `v`: 切换夹爪开关

### 右臂控制
- `i/k`: X轴 前进/后退
- `j/l`: Y轴 左/右
- `u/o`: Z轴 上/下
- `p/;`: 滚转角 +/-
- `[/]`: 俯仰角 +/-
- `'/\\`: 偏航角 +/-
- `m`: 切换夹爪开关

### 升降电机控制
- `z`: 上升 (按住连续上升)
- `x`: 下降 (按住连续下降)

### 系统控制
- `空格` 或 `ESC`: 停止并退出

## 使用方法

### 1. 仅启动键盘控制
```bash
ros2 launch my_teleop_pkg bimanual_with_lifting_teleop.launch.py
```

### 2. 启动完整机器人控制 (升降电机+键盘控制)
```bash
ros2 launch my_teleop_pkg full_robot_teleop.launch.py
```

### 3. 分别启动各个节点

首先启动升降电机控制节点：
```bash
ros2 launch adora_lifting_motor_control adora_a2_max_ros2_node.launch.py
```

然后启动键盘控制节点：
```bash
ros2 run my_teleop_pkg bimanual_teleop
```

## 参数配置

### 键盘控制参数
- `pos_speed`: 位置控制速度 (默认: 0.01 m/按键)
- `rot_speed`: 旋转控制速度 (默认: 0.05 rad/按键)
- `lifting_step`: 升降步长 (默认: 10 mm/按键)
- `max_lifting_height`: 最大升降高度 (默认: 700 mm for A2 MAX)

### 升降电机参数
- `dev`: 串口设备 (默认: /dev/ttyACM0)
- `baud`: 波特率 (默认: 19200)
- `max_lifting_distance`: 最大行程 (A2 MAX: 700mm, A2 PRO: 900mm)
- `robot_name`: 机器人型号 (ADORA_A2_MAX 或 ADORA_A2_PRO)

## 话题接口

### 发布话题
- `/left_arm/pose_cmd` (geometry_msgs/Pose): 左臂位姿命令
- `/left_arm/gripper_cmd` (std_msgs/Bool): 左臂夹爪命令
- `/right_arm/pose_cmd` (geometry_msgs/Pose): 右臂位姿命令
- `/right_arm/gripper_cmd` (std_msgs/Bool): 右臂夹爪命令
- `/adora/lifting_motor/cmd` (std_msgs/UInt32): 升降电机位置命令 (单位: mm)

### 订阅话题
- `/adora/lifting_motor/states` (std_msgs/UInt32): 升降电机当前位置 (单位: mm)

## 注意事项

1. 使用前确保升降电机串口设备连接正常
2. 确保用户有串口设备访问权限：`sudo usermod -a -G dialout $USER`
3. 升降电机的零位位于靠近电机侧
4. 升降范围受限于机器人型号：A2 MAX (700mm), A2 PRO (900mm)
5. 按键控制是增量式的，连续按键可实现连续运动

## 故障排除

### 串口连接问题
```bash
# 检查设备是否存在
ls -la /dev/ttyACM*

# 检查用户权限
groups $USER | grep dialout

# 如果没有权限，添加用户到dialout组
sudo usermod -a -G dialout $USER
# 然后重新登录
```

### 查看实时状态
```bash
# 查看升降电机状态
ros2 topic echo /adora/lifting_motor/states

# 手动控制升降电机
ros2 topic pub -r 10 /adora/lifting_motor/cmd std_msgs/msg/UInt32 "{data: 300}"
```

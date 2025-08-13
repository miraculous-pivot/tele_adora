# VR Arm Sync Package

这个包实现了VR控制器与机械臂的位姿同步功能，集成了机械臂的初始化和控制逻辑。

## 功能描述

- 自动初始化左右机械臂
- 订阅VR控制器的模拟量输入（`/left_analog`, `/right_analog`）
- 订阅VR控制器的位姿（`/left_transform`, `/right_transform`）  
- 直接控制机械臂位姿和gripper
- 发布机械臂当前位姿（`/left_arm/pose_cmd`, `/right_arm/pose_cmd`）
- 发布gripper状态（`/left_arm/gripper_cmd`, `/right_arm/gripper_cmd`）

## 工作原理

1. **校准功能**: 当`/left_analog`或`/right_analog`话题中的`rot_z`字段为1.0时，触发对应机械臂的校准
2. **gripper控制**: 当`/left_analog`或`/right_analog`话题中的`rot_y`字段为1.0时，切换gripper状态（开/关），两次操作间隔至少2秒
3. **位姿同步**: 校准完成后，根据VR控制器位姿相对于基准位姿的变化，直接控制机械臂位姿

## 使用方法

### 初始化系统
```bash
cd /home/feng/tele_adora/VR_service/src/vr_arm_sync
./init.sh
```

初始化脚本会自动：
- 检测和激活CAN接口
- 检查设备版本
- 编译包
- 设置环境

### 启动节点
```bash
ros2 launch vr_arm_sync vr_arm_sync.launch.py
```

或者直接运行节点：
```bash
ros2 run vr_arm_sync vr_arm_sync_node
```

### 校准和控制流程
1. 确保机械臂已经初始化成功
2. 将VR控制器移动到期望的起始位置
3. 通过VR控制器使`/left_analog`或`/right_analog`的`rot_z`字段变为1.0来触发校准
4. 校准完成后，移动VR控制器，机械臂将同步跟随
5. 通过VR控制器使`rot_y`字段变为1.0来控制gripper开关

## 话题说明

### 订阅的话题
- `/left_analog` (my_custom_interfaces/PosRot): 左控制器模拟量输入
  - `rot_z = 1.0`: 触发校准
  - `rot_y = 1.0`: 控制gripper
- `/right_analog` (my_custom_interfaces/PosRot): 右控制器模拟量输入
  - `rot_z = 1.0`: 触发校准
  - `rot_y = 1.0`: 控制gripper
- `/left_transform` (my_custom_interfaces/PosRot): 左控制器位姿
- `/right_transform` (my_custom_interfaces/PosRot): 右控制器位姿

### 发布的话题
- `/left_arm/pose_cmd` (geometry_msgs/Pose): 左机械臂当前位姿
- `/right_arm/pose_cmd` (geometry_msgs/Pose): 右机械臂当前位姿
- `/left_arm/gripper_cmd` (std_msgs/Bool): 左gripper状态
- `/right_arm/gripper_cmd` (std_msgs/Bool): 右gripper状态

## 硬件配置

默认CAN配置：
- 左机械臂: can0
- 右机械臂: can1

如需修改，请编辑节点代码中的`LEFT_ARM_CAN`和`RIGHT_ARM_CAN`常量。

## 注意事项

- 确保在运行前已正确连接和配置CAN接口
- 使用`./init.sh`脚本进行完整的系统初始化
- Gripper控制有2秒间隔限制，避免频繁操作
- 节点会自动处理机械臂的初始化和关闭
- 坐标变换采用简化的线性映射，实际应用中可能需要调整`POS_SCALE_FACTOR`和`ROT_SCALE_FACTOR`

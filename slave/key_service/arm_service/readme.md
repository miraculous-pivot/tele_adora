```shell
bash ./script/init.sh
colcon  build  
source ./install/setup.bash
ros2  run arm_service ctrl_node
```

## 注意这里默认can0为左臂，can1为右臂

## 注意要根据机械臂情况调整缩放因子
POS_SCALE_FACTOR
ROT_SCALE_FACTOR

## 注意ctrl+C会失能它
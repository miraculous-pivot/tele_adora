```shell
colcon  build  
source ./install/setup.bash
ros2  run my_teleop_pkg bimanual_teleop
```

## operation guide
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
# Tele-Adora 遥操作系统

## ⚠️ 首次使用必读

**首次克隆或下载项目后，必须先设置脚本执行权限：**

```bash
# 在项目根目录运行
bash setup_permissions.sh
```

如果上述命令无法运行，请先手动设置权限：
```bash
chmod +x setup_permissions.sh
./setup_permissions.sh
```

详细权限设置说明请参考：[PERMISSIONS_SETUP.md](./PERMISSIONS_SETUP.md)

## 快速启动

### 方法一：使用快速启动脚本
```bash
./start.sh
```

### 方法二：直接使用主脚本
```bash
cd script
./main_setup.sh
```

## 系统架构

本系统支持两种设备类型：

1. **本体 (Robot Body)** - 机器人本体端
   - 键盘控制服务
   - VR控制服务

2. **遥操作设备 (Teleoperation Device)** - 控制端
   - 足部控制器
   - 键盘设备

## 目录结构

```
tele_adora-master/
├── start.sh                    # 快速启动脚本
├── script/                     # 所有启动脚本
│   ├── main_setup.sh          # 主启动脚本（交互式菜单）
│   ├── setup_keyboard.sh      # 键盘设备控制端
│   ├── setup_keyservice.sh    # 键盘控制服务端
│   ├── setup_vrservice.sh     # VR控制服务端
│   ├── setup_step.bash        # 足部控制器控制端
│   └── README.md              # 详细使用说明
├── leader/                     # 控制端模块
├── slave/                      # 机器人本体端模块
└── reference/                  # 参考文档
```

## 使用说明

1. 运行 `./start.sh` 启动系统
2. 根据提示选择设备类型和控制方式
3. 确认选择后系统将自动启动对应的服务

详细使用说明请查看 `script/README.md`

## 注意事项

- 需要 sudo 权限访问串口设备
- 确保 ROS2 环境已正确配置
- 确保所有硬件设备已正确连接

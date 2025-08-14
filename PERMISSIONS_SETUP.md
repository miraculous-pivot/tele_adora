# Tele-Adora 项目权限设置指南

## 问题说明

当您从 git 仓库克隆项目或者将项目传输到其他机器时，所有的 `.sh` 脚本文件可能会失去执行权限，导致无法直接运行脚本。

## 快速解决方案

我们提供了一个一键设置所有脚本权限的工具：

### 方法1：使用权限设置脚本（推荐）

```bash
# 在项目根目录下运行
cd /path/to/tele_adora
bash setup_permissions.sh
```

这个脚本会自动为项目中的所有 `.sh` 文件添加执行权限。

### 方法2：手动设置（如果自动脚本无法运行）

如果 `setup_permissions.sh` 本身没有执行权限，请先手动设置：

```bash
chmod +x setup_permissions.sh
./setup_permissions.sh
```

### 方法3：完全手动设置

如果需要手动设置所有权限：

```bash
# 主要脚本
chmod +x start.sh
chmod +x script/*.sh

# 服务脚本  
chmod +x slave/key_service/arm_service/script/*.sh
chmod +x slave/VR_service/script/*.sh

# 视频相关脚本
chmod +x slave/video/webrtc_pub/*.sh
chmod +x leader/key_board/*.sh

# 开发工具脚本
chmod +x dev/piper_sdk/*.sh
chmod +x dev/piper_sdk/piper_sdk/*.sh
```

## 验证权限设置

设置完权限后，您可以验证主脚本是否可以运行：

```bash
./start.sh
# 或者
bash start.sh
```

## 项目中的主要脚本

- `start.sh` - 主启动脚本
- `setup_permissions.sh` - 权限设置工具
- `script/main_setup.sh` - 主设置脚本
- `script/setup_keyservice.sh` - 键盘控制服务
- `script/setup_vrservice.sh` - VR控制服务

## 注意事项

1. **首次使用必须运行权限设置**
2. **每次从 git 克隆后都需要重新设置权限**
3. **如果遇到 "Permission denied" 错误，请检查脚本权限**

## Git 配置建议

为了避免权限问题，建议在 git 配置中保留文件权限：

```bash
git config core.filemode true
```

但请注意，这个设置在某些系统上可能不起作用，所以最好的做法还是使用 `setup_permissions.sh` 脚本。

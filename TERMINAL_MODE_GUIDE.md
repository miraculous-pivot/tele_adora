# 终端模式控制功能说明

## 🎯 新增功能

现在支持通过参数控制是否显示终端窗口！

### 📋 使用方法

#### 1. 默认模式（后台运行，不弹出终端）
```bash
# 方法1：使用快速启动脚本
bash start.sh

# 方法2：直接使用主脚本
cd script
bash main_setup.sh

# 方法3：直接使用服务脚本
cd script
bash setup_keyservice.sh
# 或
bash setup_vrservice.sh
```

#### 2. 显示终端模式（弹出终端窗口）
```bash
# 方法1：使用快速启动脚本
bash start.sh --show-terminals

# 方法2：直接使用主脚本
cd script
bash main_setup.sh --show-terminals

# 方法3：直接使用服务脚本
cd script
bash setup_keyservice.sh --show-terminals
# 或
bash setup_vrservice.sh --show-terminals
```

#### 3. 短参数形式
```bash
# 也支持短参数 -t
bash start.sh -t
bash main_setup.sh -t
bash setup_keyservice.sh -t
bash setup_vrservice.sh -t
```

## 🔄 模式对比

### 默认模式（后台运行）
```
模式: 后台运行 (使用 --show-terminals 或 -t 参数显示终端窗口)

后台启动: LIFTING MOTOR CONTROL (日志: /tmp/tele_adora_LIFTING_MOTOR_CONTROL.log)
后台启动: CHASSIS CONTROL (日志: /tmp/tele_adora_CHASSIS_CONTROL.log)
后台启动: GIMBAL CONTROL (日志: /tmp/tele_adora_GIMBAL_CONTROL.log)
...
```

**优势:**
- ✅ 不会弹出多个终端窗口，保持桌面清洁
- ✅ 所有服务在后台运行
- ✅ 每个服务的输出保存到单独的日志文件
- ✅ 适合生产环境使用

### 显示终端模式
```
模式: 显示终端窗口

[弹出多个gnome-terminal窗口]
- LIFTING MOTOR CONTROL
- CHASSIS CONTROL  
- GIMBAL CONTROL
- ARM CONTROL
- REALSENSE CAMERAS
- ORBBEC CAMERA
- ZED CAMERA
- WEB RTC
```

**优势:**
- ✅ 可以实时查看每个服务的输出
- ✅ 方便调试和监控
- ✅ 可以与各个服务进行交互
- ✅ 适合开发和调试环境

## 📁 日志文件位置

当使用默认模式（后台运行）时，每个服务的日志保存在：

```
/tmp/tele_adora_LIFTING_MOTOR_CONTROL.log
/tmp/tele_adora_CHASSIS_CONTROL.log
/tmp/tele_adora_GIMBAL_CONTROL.log
/tmp/tele_adora_ARM_CONTROL.log
/tmp/tele_adora_REALSENSE_CAMERAS.log
/tmp/tele_adora_ORBBEC_CAMERA.log
/tmp/tele_adora_ZED_CAMERA.log
/tmp/tele_adora_WEB_RTC.log
```

### 查看日志示例
```bash
# 查看特定服务的日志
tail -f /tmp/tele_adora_ARM_CONTROL.log

# 查看所有服务的日志
tail -f /tmp/tele_adora_*.log
```

## 🛠️ 技术实现

### 核心函数
```bash
run_service() {
    local title="$1"
    local working_dir="$2" 
    local command="$3"
    
    if [ "$SHOW_TERMINALS" = true ]; then
        # 显示终端窗口
        gnome-terminal --title="$title" --working-directory="$working_dir" -- bash -c "$command"
    else
        # 后台运行，输出到日志文件
        local log_file="/tmp/tele_adora_${title// /_}.log"
        echo "后台启动: $title (日志: $log_file)"
        cd "$working_dir"
        nohup bash -c "$command" > "$log_file" 2>&1 &
        cd - > /dev/null
    fi
}
```

### 参数传递链
```
start.sh → main_setup.sh → setup_keyservice.sh/setup_vrservice.sh
   ↓            ↓                    ↓
 "$@"      TERMINAL_ARG        SHOW_TERMINALS
```

## 🎯 使用建议

- **生产环境**: 使用默认模式（后台运行）
- **开发调试**: 使用 `--show-terminals` 模式
- **首次测试**: 建议使用 `--show-terminals` 查看服务启动情况
- **稳定运行**: 使用默认模式，通过日志文件监控

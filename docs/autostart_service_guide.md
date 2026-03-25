# ROS 2 Vision 系统自启动配置指南

## 概述

本指南介绍如何配置 ckyf_vision ROS 2 系统在系统启动时自动运行，并在前台弹出终端窗口便于直接观察输出。

**支持的启动方式：**

| 方式 | 优点 | 缺点 | 适用场景 |
|------|------|------|---------|
| **systemd User Service** | 完整日志、可靠、支持自动重启 | 需要配置 | 生产部署、自动化环境 |
| **Desktop Autostart** | 简单快速、集成桌面环境 | 日志管理有限 | 开发环境、快速部署 |

---

## 方式 1: systemd User Service（推荐）

### 特点

✓ 开机自动启动，在前台弹出终端窗口
✓ 完整的日志记录（systemd 日志 + 应用日志）
✓ 支持自动重启（服务崩溃时）
✓ 方便的启停管理
✓ 与 graphical-session 集成

### 安装步骤

**1. 使用交互式配置工具：**

```bash
bash tools/setup_autostart_service.sh
```

选择菜单选项 [1] 或 [2]。

**2. 或直接指定参数：**

```bash
# 使用默认机型
bash tools/setup_autostart_service.sh --method systemd

# 指定机型
bash tools/setup_autostart_service.sh --method systemd --robot-type infantry_4
```

### 使用命令

**启动服务：**
```bash
systemctl --user start ckyf-vision
```

**停止服务：**
```bash
systemctl --user stop ckyf-vision
```

**查看状态：**
```bash
systemctl --user status ckyf-vision
```

**查看日志：**
```bash
# 实时日志（systemd 日志）
journalctl --user -u ckyf-vision -f

# 应用日志（更详细）
tail -f ~/.ckyf_vision_autostart/vision.log

# 查看最近 50 行
journalctl --user -u ckyf-vision -n 50 --no-pager
```

**禁用自启动：**
```bash
systemctl --user disable ckyf-vision
```

**启用自启动：**
```bash
systemctl --user enable ckyf-vision
```

### systemd 服务文件位置

```
~/.config/systemd/user/ckyf-vision.service
```

### 日志位置

```
~/.ckyf_vision_autostart/vision.log
```

### 故障排除

**问题：启动后服务立即失败**

检查日志：
```bash
journalctl --user -u ckyf-vision -n 100 --no-pager
cat ~/.ckyf_vision_autostart/vision.log
```

常见原因：
- 项目未编译：执行 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
- ROS 2 环境不完整：检查 `source install/setup.bash`
- 相机/串口连接问题：物理验证硬件连接

**问题：修改了启动参数（机型）后不生效**

需要编辑服务文件：
```bash
nano ~/.config/systemd/user/ckyf-vision.service
# 修改 Environment="ROBOT_TYPE=xxx"
systemctl --user daemon-reload
systemctl --user restart ckyf-vision
```

或重新运行配置脚本：
```bash
bash tools/setup_autostart_service.sh --uninstall
bash tools/setup_autostart_service.sh --method systemd --robot-type <new_type>
```

---

## 方式 2: Desktop Autostart

### 特点

✓ 简单快速，仅需在桌面环境中配置
✓ 集成于操作系统自启动机制
✗ 日志管理相对复杂

### 安装步骤

**1. 使用交互式配置工具：**

```bash
bash tools/setup_autostart_service.sh
```

选择菜单选项 [3]。

**2. 或直接指定参数：**

```bash
bash tools/setup_autostart_service.sh --method desktop --robot-type infantry_4
```

### 配置文件位置

```
~/.config/autostart/ckyf-vision.desktop
```

### 禁用自启动

**方式 1：GUI 方式**
- 打开 "设置" → "应用" → "启动应用程序"
- 找到 "ckyf_vision ROS 2" 并禁用

**方式 2：命令行**
```bash
rm ~/.config/autostart/ckyf-vision.desktop
```

### 日志位置

```
~/.ckyf_vision_autostart/vision.log
```

---

## 快速启动脚本

除了自启动配置外，还提供了独立的快速启动脚本，可用于手动前台运行。

### 使用方法

**使用默认机型启动：**
```bash
bash tools/quick_start_vision.sh
```

**指定机型启动：**
```bash
bash tools/quick_start_vision.sh infantry_4
```

**指定日志位置：**
```bash
bash tools/quick_start_vision.sh default --log-file /tmp/vision.log
```

### 特点

✓ 前台运行，终端输出即时可见
✓ 同时保存日志文件
✓ 自动日志轮转（保留最近 10 个）
✓ 完整的环境检查

---

## 完整配置示例

### 场景 1：实车自动启动（推荐）

```bash
# 第一次部署时
cd ~/ckyf_vision
bash tools/setup_rt_full.sh --install-tools  # 系统级优化
bash tools/setup_complete.sh                  # 交互式部署
bash tools/setup_autostart_service.sh         # 配置自启动

# 选择：systemd 方式，机型：infantry_4

# 验证部署
bash tools/check_rt_status.sh
systemctl --user status ckyf-vision

# 重启系统
sudo reboot

# 系统启动后，ROS 2 Vision 会自动在前台启动
```

### 场景 2：开发环境快速启动

```bash
# 开发阶段：手动快速启动进行调试
bash tools/quick_start_vision.sh --robot-type default

# 修改代码后
colcon build --symlink-install
bash tools/quick_start_vision.sh --robot-type default

# 或启用 systemd 服务进行后台测试
systemctl --user start ckyf-vision
journalctl --user -u ckyf-vision -f
```

### 场景 3：多机型配置

```bash
# 步兵 4 号机
bash tools/setup_autostart_service.sh --method systemd --robot-type infantry_4

# 哨兵机
bash tools/setup_autostart_service.sh --method systemd --robot-type sentry

# 切换启动时，修改服务配置
nano ~/.config/systemd/user/ckyf-vision.service
# 修改：Environment="ROBOT_TYPE=sentry"
systemctl --user daemon-reload
systemctl --user restart ckyf-vision
```

---

## 环境变量配置

### 通过 systemd 环境变量调整

编辑 `~/.config/systemd/user/ckyf-vision.service`：

```ini
[Service]
Environment="ROBOT_TYPE=infantry_4"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="CYCLONEDDS_URI=file://${HOME}/ckyf_vision/CycloneDDS.xml"
```

然后重载：
```bash
systemctl --user daemon-reload
systemctl --user restart ckyf-vision
```

### 日志级别控制

```bash
# 在 launch 文件中添加
ros2 launch robot_bringup vision.launch.py robot_type:=infantry_4 log_level:=debug
```

---

## 常见问题 (FAQ)

### Q1: 如何在实车断电重启后自动启动？

**A:** 配置 systemd user service：

```bash
bash tools/setup_autostart_service.sh --method systemd
# 选择你的机型
```

systemd 会在用户登录后自动启动（与 graphical-session 绑定）。

### Q2: 多个用户如何配置不同的自启动？

**A:** 每个用户独立配置，以用户身份运行：

```bash
# 用户 A
su - userA
bash ~/ckyf_vision/tools/setup_autostart_service.sh --method systemd

# 用户 B
su - userB
bash ~/ckyf_vision/tools/setup_autostart_service.sh --method systemd
```

### Q3: 如何在后台静默启动而不显示终端窗口？

**A:** 编辑 systemd 服务文件，修改 wrapper 脚本：

```bash
# 编辑 ~/.config/systemd/user/ckyf-vision.service
# 改为后台启动：
ExecStart=/home/USER/.local/bin/ckyf-vision-start-background.sh
```

### Q4: 启动后想动态修改参数（如机型），如何操作？

**A:**

方式 1（快速）：
```bash
# 杀死当前进程
systemctl --user stop ckyf-vision

# 修改环境变量
nano ~/.config/systemd/user/ckyf-vision.service
# 改：Environment="ROBOT_TYPE=infantry_3"

# 重启
systemctl --user daemon-reload
systemctl --user start ckyf-vision
```

方式 2（完全重配）：
```bash
bash tools/setup_autostart_service.sh --uninstall
bash tools/setup_autostart_service.sh --method systemd --robot-type infantry_3
```

### Q5: 如何查看启动失败的原因？

**A:**

```bash
# 1. 查看 systemd 日志
journalctl --user -u ckyf-vision -n 100 --no-pager

# 2. 查看应用日志
tail -f ~/.ckyf_vision_autostart/vision.log

# 3. 手动测试（排除 systemd 环境问题）
bash tools/quick_start_vision.sh --robot-type default

# 4. 检查 ROS 2 环境
ros2 doctor --report
```

### Q6: 如何在容器中使用自启动？

**A:** 容器启动脚本中调用：

```bash
# Dockerfile 或启动脚本
bash /path/to/ckyf_vision/tools/quick_start_vision.sh --robot-type default
```

（systemd 需要容器支持 systemd，通常不推荐）

---

## 性能优化建议

### 1. 与系统级优化配合

建议同时部署系统级性能优化：

```bash
# 系统级优化（一次性）
sudo bash tools/setup_rt_full.sh --install-tools

# 配置自启动（用户级）
bash tools/setup_autostart_service.sh --method systemd
```

### 2. 日志级别调整

生产环境建议降低日志输出量：

修改 `robot_bringup/launch/vision.launch.py`：

```python
# 添加参数
log_level = LaunchConfiguration('log_level', default='warn')

# 在节点配置中使用
arguments=['--ros-args', '--log-level', log_level]
```

启动时指定：
```bash
ros2 launch robot_bringup vision.launch.py robot_type:=infantry_4 log_level:=warn
```

### 3. 日志轮转管理

修改 `~/.config/systemd/user/ckyf-vision.service`：

```ini
[Service]
# 限制日志大小
StandardOutput=journal
StandardError=journal
```

systemd 会自动管理日志大小和轮转。

---

## 卸载自启动

### 使用脚本卸载（推荐）

```bash
bash tools/setup_autostart_service.sh --uninstall
```

### 手动卸载

**systemd 方式：**
```bash
systemctl --user disable ckyf-vision.service
systemctl --user stop ckyf-vision.service
rm ~/.config/systemd/user/ckyf-vision.service
systemctl --user daemon-reload
```

**Desktop 方式：**
```bash
rm ~/.config/autostart/ckyf-vision.desktop
```

**清理脚本和日志：**
```bash
rm ~/.local/bin/ckyf-vision-start.sh
rm -rf ~/.ckyf_vision_autostart/
```

---

## 附录：systemd 服务文件说明

```ini
[Unit]
# 服务描述
Description=ckyf_vision ROS 2 Vision Auto-Aim System

# 依赖关系：在网络和图形界面就绪后启动
After=network.target graphical-session.target

# 属于桌面会话
PartOf=graphical-session.target

[Service]
# 服务类型：simple 表示前台运行
Type=simple

# 启动命令
ExecStart=/home/USER/.local/bin/ckyf-vision-start.sh

# 故障重启策略
Restart=on-failure
RestartSec=5

# 环境变量
Environment="ROBOT_TYPE=default"

# 日志输出
StandardOutput=inherit
StandardError=inherit

[Install]
# 安装到桌面会话
WantedBy=graphical-session.target
```

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0.0 | 2026-03-25 | 初始版本：支持 systemd 和 desktop autostart 两种方式 |


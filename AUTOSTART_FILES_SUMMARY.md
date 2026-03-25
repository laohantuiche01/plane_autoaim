# ROS 2 Vision 自启动配置 - 文件清单

## 📦 新增文件总览

### 核心脚本

| 文件 | 说明 |
|------|------|
| `tools/setup_autostart_service.sh` | 主配置脚本，支持 systemd 和 desktop 两种启动方式 |
| `tools/quick_start_vision.sh` | 快速前台启动脚本，用于手动测试或被自启动服务调用 |

**特点：**
- ✅ 完整的参数验证和错误处理
- ✅ 交互式菜单支持
- ✅ 命令行参数支持（自动化脚本友好）
- ✅ 彩色输出便于观察
- ✅ 详细的日志记录
- ✅ 支持卸载恢复

### 文档

| 文件 | 说明 |
|------|------|
| `docs/autostart_service_guide.md` | 完整使用指南（150+ 行），包含原理、安装步骤、常见问题等 |
| `docs/autostart_quick_reference.md` | 快速参考卡片，常用命令一览 |
| `docs/examples/ckyf-vision.service.example` | systemd 服务文件示例（含详细注释） |
| `docs/examples/ckyf-vision.desktop.example` | Desktop Entry 文件示例（含详细注释） |

---

## 🎯 快速开始

### 最简单的方式（3 步）

```bash
# 1. 进入项目目录
cd ~/ckyf_vision

# 2. 运行配置脚本
bash tools/setup_autostart_service.sh

# 3. 选择 [1] systemd 和你的机型，完成！
```

下次系统启动时，ROS 2 Vision 会自动在前台启动。

### 验证配置

```bash
# 查看自启动状态
systemctl --user status ckyf-vision

# 查看日志
journalctl --user -u ckyf-vision -f
```

---

## 💻 命令参考

### 配置脚本用法

```bash
# 交互式菜单
bash tools/setup_autostart_service.sh

# 直接配置 systemd 方式
bash tools/setup_autostart_service.sh --method systemd --robot-type infantry_4

# 直接配置 desktop 方式
bash tools/setup_autostart_service.sh --method desktop --robot-type default

# 查看自启动状态
bash tools/setup_autostart_service.sh --status

# 卸载自启动
bash tools/setup_autostart_service.sh --uninstall
```

### 快速启动脚本用法

```bash
# 使用默认机型
bash tools/quick_start_vision.sh

# 指定机型
bash tools/quick_start_vision.sh infantry_4

# 指定日志位置
bash tools/quick_start_vision.sh default --log-file /tmp/vision.log
```

### systemd 管理命令

```bash
# 启动服务
systemctl --user start ckyf-vision

# 停止服务
systemctl --user stop ckyf-vision

# 重启服务
systemctl --user restart ckyf-vision

# 查看状态
systemctl --user status ckyf-vision

# 查看日志（实时）
journalctl --user -u ckyf-vision -f

# 查看日志（最近 50 行）
journalctl --user -u ckyf-vision -n 50 --no-pager

# 启用自启动
systemctl --user enable ckyf-vision

# 禁用自启动
systemctl --user disable ckyf-vision
```

---

## 📋 支持的启动方式对比

| 特性 | systemd（推荐） | Desktop Autostart |
|------|---|---|
| 前台显示终端 | ✅ | ✅ |
| 开机自动启动 | ✅ | ✅ |
| 日志记录 | ✅✅ 完整 | ✅ 基本 |
| 自动重启 | ✅ 可配置 | ❌ |
| 命令行管理 | ✅ systemctl | ❌ |
| 实时日志查看 | ✅ journalctl | ❌ |
| 配置复杂度 | 低 | 更低 |
| 生产推荐 | ✅✅✅ | ✅ |
| 开发调试 | ✅✅ | ✅✅ |

---

## 🔧 配置文件位置

启动后会自动生成以下文件：

```
~/.config/systemd/user/ckyf-vision.service    # systemd 服务配置
~/.config/autostart/ckyf-vision.desktop       # Desktop autostart 配置
~/.local/bin/ckyf-vision-start.sh            # 启动脚本
~/.ckyf_vision_autostart/vision.log           # 应用日志
```

---

## 📚 文档结构

```
docs/
├── autostart_service_guide.md          # 详细使用指南（推荐阅读）
├── autostart_quick_reference.md        # 快速参考卡片
└── examples/
    ├── ckyf-vision.service.example     # systemd 服务文件示例
    └── ckyf-vision.desktop.example     # Desktop Entry 文件示例

tools/
├── setup_autostart_service.sh          # 主配置脚本
└── quick_start_vision.sh               # 快速启动脚本
```

---

## 🚨 常见问题速查

### 启动后没有显示终端窗口？

**原因**：未检测到图形终端。

**解决**：
```bash
# 检查可用的终端
which gnome-terminal konsole xfce4-terminal xterm

# 安装缺失的终端（Ubuntu）
sudo apt install gnome-terminal
```

### 修改了机型参数，但启动时未生效？

**解决**：
```bash
# 编辑配置文件
nano ~/.config/systemd/user/ckyf-vision.service

# 修改 Environment="ROBOT_TYPE=infantry_4"

# 重载并重启
systemctl --user daemon-reload
systemctl --user restart ckyf-vision
```

### 如何禁用自启动但保留配置？

```bash
systemctl --user disable ckyf-vision
```

### 如何完全卸载所有配置？

```bash
bash tools/setup_autostart_service.sh --uninstall
```

### 想手动测试而不启用自启动？

```bash
bash tools/quick_start_vision.sh --robot-type default
```

---

## 🔍 故障排除流程

1. **查看错误信息**
   ```bash
   journalctl --user -u ckyf-vision -n 100 --no-pager
   ```

2. **查看详细日志**
   ```bash
   tail -f ~/.ckyf_vision_autostart/vision.log
   ```

3. **验证项目环境**
   ```bash
   cd ~/ckyf_vision
   source install/setup.bash
   ros2 doctor --report
   ```

4. **手动前台测试（排除 systemd 问题）**
   ```bash
   bash tools/quick_start_vision.sh --robot-type default
   ```

5. **检查硬件**
   - 相机连接
   - 串口连接
   - USB 权限

详见 `docs/autostart_service_guide.md` 的完整故障排除章节。

---

## 📝 版本信息

- **创建日期**: 2026-03-25
- **兼容版本**: ROS 2 Humble+，ckyf_vision v1.3.0+
- **支持系统**: Ubuntu 20.04+（带 systemd）
- **对应项目版本**: v1.3.2

---

## ✅ 部署检查清单

启用自启动前，请确保：

- [ ] 项目已编译：`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
- [ ] ROS 2 环境正常：`ros2 doctor --report` 无致命错误
- [ ] 硬件连接正确：相机、串口等
- [ ] 权限配置完成：`sudo bash tools/setup_rt_full.sh` 或 `bash tools/setup_complete.sh`
- [ ] 使用正确的机型参数（default/infantry_3/infantry_4/sentry）

---

更多帮助见 `docs/autostart_service_guide.md`

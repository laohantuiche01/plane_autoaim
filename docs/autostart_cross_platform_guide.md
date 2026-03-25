# 跨平台部署指南

本文档说明如何在不同用户和不同路径上正确使用自启动配置脚本。

## 🌍 支持的部署场景

### 场景 1：标准位置部署（推荐）

适用于项目位于标准位置的情况：

```bash
# 项目位置（以下任一即可）
~/ckyf_vision
~/workspace/ckyf_vision
~/ros2_ws/src/ckyf_vision
/opt/ckyf_vision
```

**部署步骤：**

```bash
cd ~/ckyf_vision  # 或你的项目位置
bash tools/setup_autostart_service.sh
```

脚本会**自动查找**项目路径，无需手动配置。

### 场景 2：自定义项目路径

如果项目位于非标准位置，需要设置环境变量：

```bash
# 方式 1：临时设置（仅当前 shell）
export CKYF_VISION_ROOT=/home/user/my_custom_path/ckyf_vision
bash ~/ckyf_vision/tools/setup_autostart_service.sh

# 方式 2：永久配置（推荐）
echo 'export CKYF_VISION_ROOT=/home/user/my_custom_path/ckyf_vision' >> ~/.bashrc
source ~/.bashrc
bash ~/ckyf_vision/tools/setup_autostart_service.sh
```

### 场景 3：多用户部署

每个用户独立配置：

```bash
# 用户 A
su - userA
cd ~/ckyf_vision
bash tools/setup_autostart_service.sh

# 用户 B
su - userB
cd ~/ckyf_vision
bash tools/setup_autostart_service.sh
```

---

## 📁 日志和缓存位置

### 遵循 XDG 基本目录规范

脚本遵循 [XDG Base Directory specification](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html)：

```bash
# 日志位置（智能选择）
${XDG_CACHE_HOME:-$HOME/.cache}/ckyf_vision_autostart/vision.log

# 通常为：
~/.cache/ckyf_vision_autostart/vision.log
```

### 自定义缓存位置

如需使用自定义缓存位置：

```bash
# 设置 XDG_CACHE_HOME
export XDG_CACHE_HOME=/path/to/custom/cache

# 然后运行配置脚本
bash tools/setup_autostart_service.sh
```

---

## 🔧 路径推导机制

### 1. 项目路径自动查找（按优先级）

启动脚本会依次尝试以下位置：

```
1. $CKYF_VISION_ROOT 环境变量（最高优先级）
2. $HOME/ckyf_vision
3. $HOME/workspace/ckyf_vision
4. $HOME/ros2_ws/src/ckyf_vision
5. /opt/ckyf_vision
```

**查找成功条件**：目录中存在 `install/setup.bash` 文件

### 2. 项目根目录动态推导

配置脚本（`setup_autostart_service.sh`）通过脚本位置推导：

```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"  # 脚本位置
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"                     # 项目根目录
```

**优势**：无需手动指定，始终相对于脚本位置准确。

---

## ⚠️ 常见问题排除

### Q1: "无法找到 ckyf_vision 项目"

**原因**：项目不在标准位置。

**解决**：

```bash
# 确认项目位置
ls /path/to/your/ckyf_vision/install/setup.bash

# 设置环境变量并重新配置
export CKYF_VISION_ROOT=/path/to/your/ckyf_vision
bash $CKYF_VISION_ROOT/tools/setup_autostart_service.sh
```

### Q2: 启动后说 "项目未编译"

**原因**：项目路径不正确或项目未编译。

**解决**：

```bash
# 验证项目
cd /path/to/ckyf_vision
ls install/setup.bash  # 应该存在

# 如未编译，执行构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Q3: 多台机器同步项目路径

使用 NFS 或共享存储时：

```bash
# 所有机器上使用相同的挂载点
mount -t nfs server:/export/ckyf_vision /home/shared/ckyf_vision

# 配置脚本会自动识别
bash /home/shared/ckyf_vision/tools/setup_autostart_service.sh
```

---

## 🐳 容器/虚拟环境部署

### Docker

```dockerfile
FROM ros:humble

# 复制项目
COPY ckyf_vision /root/ckyf_vision

# 运行配置脚本
RUN cd /root/ckyf_vision && \
    bash tools/setup_autostart_service.sh --method systemd --robot-type default

# 启动系统
CMD ["bash", "/root/.local/bin/ckyf-vision-start.sh"]
```

### 虚拟环境（venv/conda）

```bash
# 激活虚拟环境
source /path/to/venv/bin/activate

# ROS 2 环境
source /opt/ros/humble/setup.bash

# 配置自启动
cd ~/ckyf_vision
bash tools/setup_autostart_service.sh
```

---

## 📋 配置清单

部署前确保：

- [ ] 项目已编译：`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
- [ ] 项目路径有效：`ls $PROJECT_ROOT/install/setup.bash` 存在
- [ ] ROS 2 已安装：`ros2 doctor --report` 无致命错误
- [ ] 硬件连接正确：相机、串口等已连接
- [ ] 如使用自定义路径，已设置 `CKYF_VISION_ROOT` 环境变量

---

## 🔗 相关文档

- `docs/autostart_service_guide.md` - 完整使用指南
- `docs/autostart_quick_reference.md` - 快速参考
- `tools/setup_autostart_service.sh` - 配置脚本（含详细注释）
- `tools/quick_start_vision.sh` - 快速启动脚本

---

## 📞 支持

如遇问题，按以下步骤排查：

1. 查看错误日志
   ```bash
   journalctl --user -u ckyf-vision -n 100 --no-pager
   cat ${XDG_CACHE_HOME:-$HOME/.cache}/ckyf_vision_autostart/vision.log
   ```

2. 验证项目环境
   ```bash
   cd $CKYF_VISION_ROOT
   source install/setup.bash
   ros2 doctor --report
   ```

3. 手动前台测试
   ```bash
   bash $CKYF_VISION_ROOT/tools/quick_start_vision.sh --robot-type default
   ```

4. 检查系统日志
   ```bash
   journalctl -e  # 系统事件日志
   ```

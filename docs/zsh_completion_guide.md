# ZSH 补全快速参考

## 概述

为 ROS 2 和 Colcon 提供 ZSH shell 命令补全，提高工作效率。基于 `python-argcomplete`。

## 安装与配置

### 快速部署

```bash
bash tools/setup_zsh_completion.sh
exec zsh  # 重启 shell 使补全生效
```

### 检查状态

```bash
bash tools/setup_zsh_completion.sh --check
```

### 通过菜单配置

```bash
bash tools/setup_complete.sh
# 选择选项 [6] "配置 ZSH 补全"
```

## 使用示例

### ROS 2 命令补全

```bash
# 列出可用命令
ros2 <TAB>

# 命令补全示例
ros2 node list <ENTER>
ros2 topic echo /camera_info <ENTER>
ros2 service call /reset_counter <TAB>
ros2 param set /my_node param_name <TAB>
```

### Colcon 命令补全

```bash
# 列出可用命令
colcon <TAB>

# 命令补全示例
colcon build --packages-select <TAB>
colcon test --packages-select <TAB>
colcon list <ENTER>
```

## 依赖

- **python3-argcomplete**：提供 argcomplete 命令补全框架
  - 自动检测并安装（首次运行脚本时）
  - 包含 `register-python-argcomplete3` 命令行工具

## 工作原理

脚本在 `~/.zshrc` 末尾添加以下配置：

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

- `register-python-argcomplete3`：扫描 Python 3 的 argparse 配置
- 动态生成 ZSH 补全规则
- `eval` 在 shell 启动时加载补全

## 故障排除

### 补全不工作

**症状**：`<TAB>` 没有显示补全列表

**解决**：
1. 检查 `python3-argcomplete` 是否安装：
   ```bash
   bash tools/setup_zsh_completion.sh --check
   ```
2. 确保 `~/.zshrc` 已配置（检查末尾是否有 argcomplete 行）
3. 重启 ZSH：`exec zsh`

### "command not found: register-python-argcomplete3"

**原因**：python3-argcomplete 未安装或路径未在 PATH 中

**解决**：
```bash
# 重新运行安装脚本
bash tools/setup_zsh_completion.sh

# 或手动安装
sudo apt install python3-argcomplete
```

### ROS 命令不可用

**原因**：未 source ROS 2 环境

**解决**：
```bash
# 若使用 direnv
cd /home/mijiao/ckyf_vision && direnv allow

# 或手动
source /home/mijiao/ckyf_vision/install/setup.zsh
```

## 文件位置

| 文件 | 说明 |
|------|------|
| `tools/setup_zsh_completion.sh` | 配置脚本 |
| `~/.zshrc` | ZSH 配置文件（自动修改） |

## 参考资源

- [Argcomplete 官方文档](https://kislyuk.github.io/argcomplete/)
- [ROS 2 官方补全指南](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [Colcon 文档](https://colcon.readthedocs.io/)

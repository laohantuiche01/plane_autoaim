# Git 身份自动配置指南

## 概述

在多台车（infantry-3、infantry-4、sentry 等）的协作开发中，每台车的提交应该带有该车的标识。本方案实现自动根据机器类型设置 git 用户名，无需手动配置。

## 使用场景

### 场景 1：单台车手动配置
```bash
# 手动配置当前仓库的 git 用户名
cd /home/mijiao/ckyf_vision
bash tools/setup_git_identity.sh

# 输出示例
# Git 身份自动配置
# ============================================================
# 机器信息：
#   主机名：infantry-3
#   标识符：infantry_3
#
# 配置本地 Git 用户...
#   用户名：robot-infantry_3
#   邮箱：  robot-infantry_3@vision-team.local
# ✅ 本地配置完成
#
# 当前 Git 配置：
#   用户名：robot-infantry_3
#   邮箱：  robot-infantry_3@vision-team.local
```

### 场景 2：自动激活 hook（推荐生产）
```bash
# 一键激活自动配置 hook
bash tools/activate_git_identity_hook.sh

# 之后每次 cd 进入 ckyf_vision 目录时自动配置
cd /home/mijiao/ckyf_vision  # 自动设置为 robot-infantry_3
```

### 场景 3：个人电脑禁用
```bash
# 个人电脑上禁用自动配置
bash tools/disable_git_identity_hook.sh

# 验证
cd /home/mijiao/ckyf_vision
git config user.name  # 应显示个人配置或 global 配置
```

## 工作原理

### 识别机器类型的优先级（简化后仅两级）

1. **环境变量 `$ROBOT_TYPE`** （最高优先）
   ```bash
   export ROBOT_TYPE=infantry_4
   bash tools/setup_git_identity.sh
   # → git 用户名：robot-infantry_4
   ```

2. **主机名直接识别** （默认）
   ```bash
   # 主机名自动识别
   # hostname = "infantry-3" → robot-infantry_3
   # hostname = "sentry-01"  → robot-sentry
   # hostname = "my-device"  → robot-my-device
   ```

## 文件说明

| 文件 | 用途 |
|------|------|
| `setup_git_identity.sh` | 核心配置脚本 |
| `set_robot_type.sh` | 机型自动检测脚本 |
| `git_identity_hook.sh` | 自动化 hook（进入项目目录时触发） |
| `activate_git_identity_hook.sh` | 一键激活脚本 |
| `disable_git_identity_hook.sh` | 禁用脚本（个人电脑用） |
| `enable_git_identity_hook.sh` | 启用脚本（恢复后用） |

## 部署方式

### 方式 1：手动配置（最灵活）

```bash
cd /home/mijiao/ckyf_vision
bash tools/setup_git_identity.sh

# 验证
git config user.name  # 应显示 robot-*
```

**优点**：完全可控
**缺点**：每台车要手动运行一次

### 方式 2：自动激活 hook（推荐生产）

```bash
# 第一次激活
bash tools/activate_git_identity_hook.sh
source ~/.bashrc

# 之后自动配置
cd /home/mijiao/ckyf_vision  # 自动生效
```

**优点**：自动化，无需手动操作
**缺点**：需要修改 shell 配置

### 方式 3：个人电脑禁用（可选）

```bash
# 禁用自动配置
bash tools/disable_git_identity_hook.sh
source ~/.bashrc

# 恢复自动配置
bash tools/enable_git_identity_hook.sh
source ~/.bashrc
```

## 配置示例

### 示例 1：Infantry-3 机型

```bash
$ bash tools/setup_git_identity.sh

机器信息：
  主机名：infantry-3
  标识符：infantry_3

配置本地 Git 用户...
  用户名：robot-infantry_3
  邮箱：  robot-infantry_3@vision-team.local
✅ 完成

提交效果：
$ git log -1 --format='%an <%ae>'
robot-infantry_3 <robot-infantry_3@vision-team.local>
```

### 示例 2：Sentry 机型

```bash
$ bash tools/setup_git_identity.sh

机器信息：
  主机名：sentry
  标识符：sentry

配置本地 Git 用户...
  用户名：robot-sentry
  邮箱：  robot-sentry@vision-team.local
✅ 完成

提交效果：
$ git log -1 --format='%an <%ae>'
robot-sentry <robot-sentry@vision-team.local>
```

## 自定义配置

### 修改邮箱域名

通过环境变量：

```bash
export MAIL_DOMAIN=mycompany.com
bash tools/setup_git_identity.sh
```

或编辑脚本第 12 行：

```bash
MAIL_DOMAIN="${MAIL_DOMAIN:-mycompany.com}"
```

### 使用 ROBOT_TYPE 环境变量

```bash
export ROBOT_TYPE=sentry
bash tools/setup_git_identity.sh
# → git 用户名：robot-sentry
```

## 故障排除

### 问题 1：自动识别失效

**症状**：运行脚本后 git 用户名不符合预期

**解决**：

```bash
# 检查主机名
hostname -s

# 手动指定
export ROBOT_TYPE=infantry_4
bash tools/setup_git_identity.sh
```

### 问题 2：Hook 不自动运行

**症状**：cd 进入项目后 git 用户名没有更新

**解决**：

```bash
# 检查 hook 是否已添加
grep "git_identity_hook" ~/.bashrc

# 重新加载配置
source ~/.bashrc

# 验证
cd /home/mijiao/ckyf_vision
git config user.name  # 应显示 robot-*
```

### 问题 3：权限问题

**症状**：运行脚本时报权限错误

**解决**：

```bash
chmod +x tools/*.sh
```

## 验证配置

### 查看当前配置

```bash
cd /home/mijiao/ckyf_vision
git config --local user.name
git config --local user.email
```

### 查看提交历史

```bash
# 查看最近 5 个提交的作者
git log -5 --format='%an <%ae>'

# 查看特定提交
git show e0c28ad --format='%an <%ae>' | head -1
```

## 最佳实践

1. **新车首先配置身份**
   ```bash
   cd /home/mijiao/ckyf_vision
   bash tools/setup_git_identity.sh
   ```

2. **启用自动 hook**
   ```bash
   bash tools/activate_git_identity_hook.sh
   ```

3. **定期验证提交身份**
   ```bash
   git log --oneline -10 --format='%an'
   ```

4. **个人电脑可禁用**
   ```bash
   bash tools/disable_git_identity_hook.sh
   ```

## 常见提交身份

```
robot-infantry_3      (Infantry 3 号步兵)
robot-infantry_4      (Infantry 4 号步兵)
robot-sentry          (哨兵)
robot-outpost         (前哨站)
robot-default         (测试环境)
```

---

**文档版本**: 1.0
**最后更新**: 2026-03-24
**维护者**: AI Assistant (Claude Haiku 4.5)

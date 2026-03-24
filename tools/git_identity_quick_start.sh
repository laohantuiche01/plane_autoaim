#!/bin/bash
# Git 身份配置快速参考
# 用法: bash tools/git_identity_quick_start.sh

cat << 'EOF'
╔════════════════════════════════════════════════════════════════════════════╗
║              Git 身份自动配置 - 快速参考                                   ║
╚════════════════════════════════════════════════════════════════════════════╝

【当前设备】
  主机名: $(hostname)
  用户: $USER

【立即配置】(推荐)
  $ bash tools/setup_git_identity.sh
  ✓ 自动识别设备类型
  ✓ 设置本地 git 用户名
  ✓ 使用格式: robot-<机型>

【自动激活】(生产环境推荐)
  $ bash tools/activate_git_identity_hook.sh
  ✓ 一次性配置
  ✓ 之后每次 cd 进项目时自动激活
  ✓ 无需手动操作

【配置示例】

  Step 1️⃣  手动配置（适合偶尔使用）
  ─────────────────────────────────────
  $ cd /home/mijiao/ckyf_vision
  $ bash tools/setup_git_identity.sh
  ✓ Git 用户名 → robot-infantry_3
  ✓ 立即生效

  Step 2️⃣  自动激活（适合生产环境）
  ─────────────────────────────────────
  $ bash tools/activate_git_identity_hook.sh
  $ source ~/.bashrc  (或 ~/.zshrc)
  ✓ 以后自动配置
  $ cd /home/mijiao/ckyf_vision
  ✓ Git 用户名已自动设置

  Step 3️⃣  验证配置
  ─────────────────────────────────────
  $ git config user.name
  robot-infantry_3

  $ git log --oneline -1 --format='%an'
  robot-infantry_3

【支持的机型识别】

  主机名包含          →  Git 用户名
  ──────────────────────────────────
  infantry-3          →  robot-infantry_3
  infantry-4          →  robot-infantry_4
  sentry              →  robot-sentry
  outpost             →  robot-outpost
  其他主机名          →  robot-<主机名>

【自定义配置】

  通过环境变量:
  $ export ROBOT_TYPE=infantry_4
  $ bash tools/setup_git_identity.sh
  ✓ 使用指定的机型，而非主机名识别

  通过邮箱域名:
  $ export MAIL_DOMAIN=mycompany.com
  $ bash tools/setup_git_identity.sh
  ✓ 邮箱格式: robot-<机型>@mycompany.com

【常用命令】

  查看当前 Git 配置:
  $ git config --local user.name
  $ git config --local user.email

  查看全局 Git 配置:
  $ git config --global user.name
  $ git config --global user.email

  查看历史提交的作者:
  $ git log --oneline -10 --format='%an <%ae>'

  验证某个提交的作者:
  $ git show e0c28ad --format='%an <%ae>' | head -1

【详细文档】

  完整的配置指南和故障排除:
  docs/git_identity_setup_guide.md

  脚本源码:
  - tools/setup_git_identity.sh         (核心配置)
  - tools/git_identity_hook.sh          (自动化 hook)
  - tools/activate_git_identity_hook.sh (一键激活)

【一键部署流程】

  对于新设备:

  1. 进入项目
     $ cd /home/mijiao/ckyf_vision

  2. 配置 Git 身份
     $ bash tools/setup_git_identity.sh

  3. (可选) 激活自动配置
     $ bash tools/activate_git_identity_hook.sh
     $ source ~/.bashrc

  4. 验证
     $ git config user.name
     ✓ robot-<机型>

【实际效果】

  Infantry-3 设备上的提交:
  ──────────────────────────
  commit e0c28ad (HEAD -> main, origin/main)
  Author: robot-infantry_3 <robot-infantry_3@vision-team.local>
  Date:   Mon Mar 24 22:32:10 2026 +0800

      [MOD] 系统级性能优化集成：jemalloc + mlockall

  Sentry 设备上的提交:
  ──────────────────────────
  commit 5caba51
  Author: robot-sentry <robot-sentry@vision-team.local>
  Date:   Mon Mar 24 22:33:20 2026 +0800

      [ADD] 新增性能优化部署和验证工具

【常见问题】

  Q: 为什么我的用户名是 robot-<完整主机名>?
  A: 主机名未能匹配预定义列表。编辑 setup_git_identity.sh 添加映射
     或使用 export ROBOT_TYPE=<机型> 手动指定

  Q: 如何在多个项目间使用不同的 Git 身份?
  A: 只需进入 ckyf_vision，脚本会自动配置本地仓库
     其他项目的 Git 身份不受影响

  Q: 我想为所有项目使用相同的 Git 身份
  A: 运行 bash tools/setup_git_identity.sh . --global
     注意这会影响整个系统的 Git 配置

  Q: Hook 自动激活后，如何禁用?
  A: 编辑 ~/.bashrc 或 ~/.zshrc，注释掉 git_identity_hook 相关行

【支持与反馈】

  文档: docs/git_identity_setup_guide.md
  脚本: tools/setup_git_identity.sh

════════════════════════════════════════════════════════════════════════════
EOF

#!/bin/bash
# Git 身份自动配置一键激活脚本
# 用法：bash tools/activate_git_identity_hook.sh
# 功能：将 Git 身份配置 hook 自动添加到 ~/.bashrc 和 ~/.zshrc

set -e

HOOK_SOURCE="/home/mijiao/ckyf_vision/tools/git_identity_hook.sh"
BASHRC="$HOME/.bashrc"
ZSHRC="$HOME/.zshrc"

echo "======================================================================="
echo "   Git 身份自动配置激活"
echo "======================================================================="
echo ""

# 检查源文件是否存在
if [ ! -f "$HOOK_SOURCE" ]; then
    echo "❌ 错误：找不到 hook 源文件"
    echo "   $HOOK_SOURCE"
    exit 1
fi

# 配置 .bashrc
echo "🔧 配置 ~/.bashrc..."
if [ -f "$BASHRC" ]; then
    if ! grep -q "git_identity_hook.sh" "$BASHRC"; then
        cat >> "$BASHRC" <<EOF

# Git 身份自动配置（ckyf_vision）- $(date '+%Y-%m-%d %H:%M:%S')
if [ -f "$HOOK_SOURCE" ]; then
    source "$HOOK_SOURCE"
fi
EOF
        echo "  ✅ 已添加 hook 到 ~/.bashrc"
    else
        echo "  ⏭️  hook 已存在于 ~/.bashrc"
    fi
else
    echo "  ℹ️  ~/.bashrc 不存在，跳过"
fi

# 配置 .zshrc
echo "🔧 配置 ~/.zshrc..."
if [ -f "$ZSHRC" ]; then
    if ! grep -q "git_identity_hook.sh" "$ZSHRC"; then
        cat >> "$ZSHRC" <<EOF

# Git 身份自动配置（ckyf_vision）- $(date '+%Y-%m-%d %H:%M:%S')
if [ -f "$HOOK_SOURCE" ]; then
    source "$HOOK_SOURCE"
fi
EOF
        echo "  ✅ 已添加 hook 到 ~/.zshrc"
    else
        echo "  ⏭️  hook 已存在于 ~/.zshrc"
    fi
else
    echo "  ℹ️  ~/.zshrc 不存在，跳过"
fi

echo ""
echo "======================================================================="
echo "   ✅ 激活完成！"
echo "======================================================================="
echo ""
echo "📌 后续步骤："
echo ""
echo "  1. 重新加载 shell 配置："
echo "     source ~/.bashrc  # 或 source ~/.zshrc"
echo ""
echo "  2. 进入项目目录测试："
echo "     cd /home/mijiao/ckyf_vision"
echo ""
echo "  3. 查看自动配置的 Git 用户名："
echo "     git config user.name"
echo ""
echo "  4. 验证提交时使用的身份："
echo "     git log --oneline -1 --format='%an <%ae>'"
echo ""
echo "======================================================================="
echo ""

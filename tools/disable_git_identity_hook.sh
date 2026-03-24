#!/bin/bash
# Git 身份自动配置禁用脚本
# 用于个人电脑，删除 hook 配置
# 用法: bash tools/disable_git_identity_hook.sh

set -e

BASHRC="$HOME/.bashrc"
ZSHRC="$HOME/.zshrc"
HOOK_SOURCE_LINE="git_identity_hook.sh"

echo "======================================================================="
echo "   Git 身份自动配置 - 禁用脚本"
echo "======================================================================="
echo ""

# 从 .bashrc 中删除
if [ -f "$BASHRC" ]; then
    if grep -q "$HOOK_SOURCE_LINE" "$BASHRC"; then
        echo "🔧 从 ~/.bashrc 中删除 hook..."
        # 删除包含 hook 的一整块（从空行到下一个空行）
        sed -i "/# Git 身份自动配置/,/^$/d" "$BASHRC"
        echo "✅ 删除完成"
    else
        echo "ℹ️  ~/.bashrc 中未找到 hook配置"
    fi
else
    echo "ℹ️  ~/.bashrc 不存在"
fi

# 从 .zshrc 中删除
if [ -f "$ZSHRC" ]; then
    if grep -q "$HOOK_SOURCE_LINE" "$ZSHRC"; then
        echo "🔧 从 ~/.zshrc 中删除 hook..."
        sed -i "/# Git 身份自动配置/,/^$/d" "$ZSHRC"
        echo "✅ 删除完成"
    else
        echo "ℹ️  ~/.zshrc 中未找到 hook配置"
    fi
else
    echo "ℹ️  ~/.zshrc 不存在"
fi

echo ""
echo "======================================================================="
echo "   ✅ 禁用完成！"
echo "======================================================================="
echo ""
echo "📋 后续步骤："
echo ""
echo "  1. 重新加载 shell 配置以使更改生效："
echo "     source ~/.bashrc  (或 source ~/.zshrc)"
echo ""
echo "  2. 验证禁用是否成功："
echo "     cd /home/mijiao/ckyf_vision"
echo "     git config user.name  # 应该是你的个人 git 配置或 global 配置"
echo ""
echo "  3. 打断后，需要手动设置 git 身份时："
echo "     bash /home/mijiao/ckyf_vision/tools/setup_git_identity.sh"
echo ""
echo "======================================================================="
echo ""
echo "💡 提示: 如果想要重新开启 hook，运行："
echo "  bash /home/mijiao/ckyf_vision/tools/enable_git_identity_hook.sh"
echo ""

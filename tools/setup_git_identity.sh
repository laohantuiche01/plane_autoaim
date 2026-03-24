#!/bin/bash
# Git 身份自动配置脚本
# 根据主机名或机器标识自动设置 git 用户名和邮箱
# 用法：bash tools/setup_git_identity.sh
# 或添加到 ~/.bashrc / ~/.zshrc 中自动运行

set -e

# ============================================================
# 配置
# ============================================================

# 邮箱域名（可根据需要修改）
MAIL_DOMAIN="${MAIL_DOMAIN:-vision-team.local}"

# 项目根目录
PROJECT_ROOT="${1:-.}"

# 操作模式（--set 配置，--get-robot-type 获取机型）
OPERATION_MODE="${OPERATION_MODE:-set}"

# ============================================================
# 函数：提取主机标识（简化逻辑：仅环境变量和主机名）
# ============================================================
get_hostname_identifier() {
    # 优先级 1：环境变量 $ROBOT_TYPE
    if [ -n "$ROBOT_TYPE" ]; then
        echo "$ROBOT_TYPE"
        return 0
    fi

    # 优先级 2：直接使用主机名（无其他依赖）
    local hostname=$(hostname -s | tr '[:upper:]' '[:lower:]')
    echo "$hostname"
    return 0
}

# ============================================================
# 函数：检查 git 仓库
# ============================================================
is_git_repo() {
    git rev-parse --is-inside-work-tree >/dev/null 2>&1
}

# ============================================================
# 函数：配置 git 用户
# ============================================================
configure_git_user() {
    local identifier=$(get_hostname_identifier)
    local git_user="robot-${identifier}"
    local git_email="${git_user}@${MAIL_DOMAIN}"

    if is_git_repo; then
        # 本地仓库配置
        echo "🔧 配置本地 Git 用户..."
        echo "  用户名：$git_user"
        echo "  邮箱：  $git_email"

        git config user.name "$git_user"
        git config user.email "$git_email"

        echo "✅ 本地配置完成"
    else
        echo "❌ 当前目录不是 Git 仓库"
        echo "   尝试配置全局 Git 用户..."
    fi

    # 全局配置（如果指定了 --global）
    if [ "$2" = "--global" ]; then
        echo "🔧 配置全局 Git 用户..."
        git config --global user.name "$git_user"
        git config --global user.email "$git_email"
        echo "✅ 全局配置完成"
    fi

    # 显示配置结果
    echo ""
    echo "📋 当前 Git 配置："
    echo "  用户名：$(git config user.name)"
    echo "  邮箱：  $(git config user.email)"
}

# ============================================================
# 函数：显示机型信息
# ============================================================
show_info() {
    echo "======================================================================="
    echo "   Git 身份自动配置"
    echo "======================================================================="
    echo ""
    echo "🤖 机器信息："
    echo "  主机名：$(hostname)"
    echo "  主机ID：$(hostname -s)"
    echo "  标识符：$(get_hostname_identifier)"
    echo ""
}

# ============================================================
# 主程序
# ============================================================
main() {
    # 检查是否是获取模式
    if [[ "$2" == "--get-robot-type" ]] || [[ "$1" == "--get-robot-type" ]]; then
        get_hostname_identifier
        return 0
    fi

    show_info
    configure_git_user "$@"

    echo ""
    echo "======================================================================="
}

# 运行主程序
main "$@"

#!/bin/bash
# ROS 2 / Colcon ZSH 补全配置脚本（基于 python-argcomplete）
# 用法：bash tools/setup_zsh_completion.sh [--check]
# 功能：检查并配置 python-argcomplete，为 ros2/colcon 提供补全

set -e

# 颜色定义
RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
CYAN=$'\033[0;36m'
NC=$'\033[0m'

print_header() {
    echo -e "\n${CYAN}========================================${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}========================================${NC}\n"
}

print_status() {
    if [ $1 -eq 0 ]; then
        echo -e "  ${GREEN}✓${NC} $2"
    else
        echo -e "  ${RED}✗${NC} $2"
    fi
}

print_info() {
    echo -e "  ${CYAN}ℹ${NC} $1"
}

print_warn() {
    echo -e "  ${YELLOW}⚠${NC} $1"
}

# ============================================================
# 1. 检查 python-argcomplete
# ============================================================
check_argcomplete() {
    print_header "Python-Argcomplete 检查"

    # 检查 register-python-argcomplete3 命令
    if command -v register-python-argcomplete3 &>/dev/null; then
        print_status 0 "register-python-argcomplete3 已安装"
        return 0
    fi

    print_status 1 "register-python-argcomplete3 未安装"
    print_info "需要安装 python3-argcomplete 包"
    return 1
}

# ============================================================
# 2. 安装 python-argcomplete
# ============================================================
install_argcomplete() {
    print_header "安装 Python-Argcomplete"

    echo "将执行以下命令："
    echo "  sudo apt install python3-argcomplete"
    echo ""
    read -p "是否继续? [y/N] " -n 1 -r
    echo ""

    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_warn "跳过安装"
        return 1
    fi

    sudo apt update
    sudo apt install -y python3-argcomplete

    if command -v register-python-argcomplete3 &>/dev/null; then
        print_status 0 "python3-argcomplete 安装成功"
        return 0
    else
        print_status 1 "安装失败"
        return 1
    fi
}

# ============================================================
# 3. 配置 .zshrc
# ============================================================
configure_zshrc() {
    print_header "配置 ~/.zshrc"

    local zshrc="$HOME/.zshrc"
    local marker="# ROS 2 / Colcon 补全（argcomplete，自动配置）"

    # 检查是否已配置
    if grep -q "register-python-argcomplete3 ros2" "$zshrc" 2>/dev/null; then
        print_status 0 "补全已在 ~/.zshrc 中配置"
        return 0
    fi

    # 添加配置
    cat >> "$zshrc" << 'EOF'

# ROS 2 / Colcon 补全（argcomplete，自动配置）
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
EOF

    print_status 0 "已添加补全配置至 ~/.zshrc"
    print_warn "请重新启动 zsh 或执行 'exec zsh' 使补全生效"
}

# ============================================================
# 4. 快速检查模式
# ============================================================
check_mode() {
    print_header "ROS 2 ZSH 补全检查"

    local all_ok=true

    # 检查 python-argcomplete
    if ! check_argcomplete; then
        all_ok=false
        print_info "运行此脚本安装：bash tools/setup_zsh_completion.sh"
    fi

    # 检查 .zshrc 配置
    if grep -q "register-python-argcomplete3 ros2" "$HOME/.zshrc" 2>/dev/null; then
        print_status 0 "~/.zshrc 已配置 ROS 补全"
    else
        print_status 1 "~/.zshrc 未配置 ROS 补全"
        all_ok=false
    fi

    if [ "$all_ok" = true ]; then
        echo -e "\n${GREEN}✓ 补全已就绪${NC}"
        return 0
    else
        echo -e "\n${YELLOW}⚠ 补全未完全配置${NC}"
        return 1
    fi
}

# ============================================================
# 主函数
# ============================================================
main() {
    # 检查参数
    if [ "$1" = "--check" ]; then
        check_mode
        exit $?
    fi

    # 完整部署流程
    if ! check_argcomplete; then
        if ! install_argcomplete; then
            echo -e "\n${RED}配置失败${NC}"
            exit 1
        fi
    fi

    configure_zshrc

    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ ROS 补全配置完成！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "下一步："
    echo "  1. 重启 zsh: exec zsh"
    echo "  2. 测试补全: ros2 <TAB>  或  colcon <TAB>"
    echo ""
}

# ============================================================
# 执行主函数
# ============================================================
main "$@"

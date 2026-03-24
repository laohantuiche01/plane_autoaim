#!/bin/bash
# ckyf_vision 项目完整初始化脚本（一站式配置）
# 用法：bash tools/setup_complete.sh
# 功能：交互式菜单，支持基础依赖、RT优化、环境配置等一键部署

set -e

# ============================================================
# 颜色定义
# ============================================================
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============================================================
# 工具函数
# ============================================================
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

check_command() {
    if command -v "$1" &>/dev/null; then
        echo -e "  ${GREEN}[已安装]${NC} $1"
        return 0
    else
        echo -e "  ${RED}[未安装]${NC} $1"
        return 1
    fi
}

check_package() {
    if dpkg -s "$1" &>/dev/null; then
        echo -e "  ${GREEN}[已安装]${NC} $1"
        return 0
    else
        echo -e "  ${RED}[未安装]${NC} $1"
        return 1
    fi
}

# ============================================================
# 菜单显示
# ============================================================
show_menu() {
    print_header "ckyf_vision 项目初始化向导"

    echo -e "${BLUE}请选择要执行的操作：${NC}\n"
    echo "  ${BLUE}1)${NC} 检查基础环境（仅检查，不安装）"
    echo "  ${BLUE}2)${NC} 安装基础依赖（OpenCV、Eigen、G2O 等）"
    echo "  ${BLUE}3)${NC} 安装 jemalloc 和 direnv"
    echo "  ${BLUE}4)${NC} 配置 direnv shell hook（已安装情况下）"
    echo "  ${BLUE}5)${NC} 配置 RT 优化（需要 root 权限）"
    echo "  ${BLUE}6)${NC} 完整部署（依赖安装 + RT 优化，需要 root 权限）"
    echo "  ${BLUE}0)${NC} 退出"
    echo ""
    read -p "请输入选项 (0-6): " choice
    echo ""
}

# ============================================================
# 功能实现
# ============================================================

# 1. 检查基础环境
check_env() {
    print_header "基础环境检查"

    echo "ROS 2 环境："
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "  ${GREEN}[已安装]${NC} ROS 2 $ROS_DISTRO"
    else
        echo -e "  ${RED}[未检测到]${NC} ROS 2"
    fi
    echo ""

    echo "系统依赖："
    check_package "libopencv-dev"
    check_package "libeigen3-dev"
    check_package "libg2o-dev"
    check_package "libfmt-dev"
    check_package "libtbb-dev"
    check_package "libsophus-dev"
    check_package "libjemalloc-dev"
    check_package "direnv"
    echo ""

    echo "ROS 2 包："
    check_package "ros-${ROS_DISTRO:-humble}-tf2-geometry-msgs"
    check_package "ros-${ROS_DISTRO:-humble}-cv-bridge"
    check_package "ros-${ROS_DISTRO:-humble}-rclcpp-components"
    check_package "ros-${ROS_DISTRO:-humble}-rclcpp-lifecycle"
    echo ""

    echo "中间件："
    check_command "iox-roudi"
    echo ""

    echo "SDK："
    if [ -d "/opt/MVS" ]; then
        echo -e "  ${GREEN}[已安装]${NC} MVS SDK"
    else
        echo -e "  ${RED}[未安装]${NC} MVS SDK"
    fi
    echo ""
}

# 2. 安装基础依赖
install_base_deps() {
    print_header "安装基础依赖"

    MISSING=()
    DEPS=(
        "libopencv-dev"
        "libeigen3-dev"
        "libg2o-dev"
        "libfmt-dev"
        "libtbb-dev"
        "libsophus-dev"
    )

    for dep in "${DEPS[@]}"; do
        if ! dpkg -s "$dep" &>/dev/null; then
            MISSING+=("$dep")
        fi
    done

    if [ ${#MISSING[@]} -gt 0 ]; then
        echo "将安装以下依赖："
        printf '  %s\n' "${MISSING[@]}"
        echo ""
        read -p "是否继续? [y/N] " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            sudo apt update
            sudo apt install -y "${MISSING[@]}"
            echo -e "${GREEN}✓${NC} 基础依赖安装完成"
        fi
    else
        echo -e "${GREEN}✓${NC} 所有基础依赖已安装"
    fi
    echo ""
}

# 3. 安装 jemalloc 和 direnv
install_jemalloc_direnv() {
    print_header "安装 jemalloc 和 direnv"

    MISSING=()

    if ! dpkg -s libjemalloc-dev &>/dev/null; then
        MISSING+=("libjemalloc-dev")
    fi

    if ! command -v direnv &>/dev/null; then
        MISSING+=("direnv")
    fi

    if [ ${#MISSING[@]} -gt 0 ]; then
        echo "将安装以下包："
        printf '  %s\n' "${MISSING[@]}"
        echo ""
        read -p "是否继续? [y/N] " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            sudo apt update
            sudo apt install -y "${MISSING[@]}"
            echo -e "${GREEN}✓${NC} jemalloc 和 direnv 安装完成"
        fi
    else
        echo -e "${GREEN}✓${NC} jemalloc 和 direnv 已安装"
    fi
    echo ""
}

# 4. 配置 direnv
configure_direnv() {
    print_header "配置 direnv"

    if ! command -v direnv &>/dev/null; then
        echo -e "${RED}✗${NC} direnv 未安装，请先执行选项 3"
        echo ""
        return
    fi

    # 配置 .zshrc
    ZSHRC="$HOME/.zshrc"
    if [ -f "$ZSHRC" ]; then
        if ! grep -q "eval \"\$(direnv hook zsh)\"" "$ZSHRC" 2>/dev/null; then
            echo "" >> "$ZSHRC"
            echo "# direnv hook（自动配置，$(date '+%Y-%m-%d')）" >> "$ZSHRC"
            echo "eval \"\$(direnv hook zsh)\"" >> "$ZSHRC"
            echo -e "  ${GREEN}✓${NC} direnv hook 已添加至 ~/.zshrc"
        else
            echo -e "  ${GREEN}✓${NC} direnv hook 已存在于 ~/.zshrc"
        fi
    fi

    # 配置 .bashrc
    BASHRC="$HOME/.bashrc"
    if [ -f "$BASHRC" ]; then
        if ! grep -q "eval \"\$(direnv hook bash)\"" "$BASHRC" 2>/dev/null; then
            echo "" >> "$BASHRC"
            echo "# direnv hook（自动配置，$(date '+%Y-%m-%d')）" >> "$BASHRC"
            echo "eval \"\$(direnv hook bash)\"" >> "$BASHRC"
            echo -e "  ${GREEN}✓${NC} direnv hook 已添加至 ~/.bashrc"
        else
            echo -e "  ${GREEN}✓${NC} direnv hook 已存在于 ~/.bashrc"
        fi
    fi

    # 项目的 .envrc 已经包含在版本控制中（无需脚本生成）
    PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    echo -e "  ${GREEN}✓${NC} .envrc 已包含在项目中（自动跨平台兼容）"
    echo ""
    echo -e "${YELLOW}ℹ️  请运行以激活环境：${NC}"
    echo "  cd $PROJECT_DIR && direnv allow"
    echo ""
    echo ""
}

# 5. 配置 RT 优化
configure_rt() {
    print_header "配置 RT 优化"

    if [ "$EUID" -ne 0 ]; then
        echo -e "${RED}✗${NC} RT 优化配置需要 root 权限"
        echo "请使用以下命令重新运行："
        echo "  sudo bash $(dirname "${BASH_SOURCE[0]}")/setup_complete.sh"
        echo ""
        return 1
    fi

    TARGET_USER=${SUDO_USER:-$(whoami)}
    echo "目标用户：$TARGET_USER"
    echo ""

    read -p "是否继续 RT 优化配置? [y/N] " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        return
    fi

    bash "$(dirname "${BASH_SOURCE[0]}")/setup_rt_full.sh" --install-tools
}

# 6. 完整部署
full_deploy() {
    print_header "完整部署"

    echo "此操作将执行以下步骤："
    echo "  1. 安装基础依赖（OpenCV、Eigen、G2O 等）"
    echo "  2. 安装 jemalloc 和 direnv"
    echo "  3. 配置 direnv hook 和 .envrc"
    echo "  4. 配置 RT 优化（需要 root 权限）"
    echo ""
    read -p "是否继续? [y/N] " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        return
    fi

    # 步骤 1-3 不需要 root
    install_base_deps
    install_jemalloc_direnv
    configure_direnv

    # 步骤 4 需要 root
    if [ "$EUID" -ne 0 ]; then
        echo ""
        echo -e "${YELLOW}ℹ️  RT 优化配置需要 root 权限，请使用以下命令：${NC}"
        echo "  sudo bash $(dirname "${BASH_SOURCE[0]}")/setup_rt_full.sh --install-tools"
        echo ""
    else
        configure_rt
    fi

    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ 部署流程完成！${NC}"
    echo -e "${GREEN}========================================${NC}"
}

# ============================================================
# 主菜单循环
# ============================================================
main() {
    while true; do
        show_menu

        case $choice in
            1)
                check_env
                ;;
            2)
                install_base_deps
                ;;
            3)
                install_jemalloc_direnv
                ;;
            4)
                configure_direnv
                ;;
            5)
                configure_rt
                ;;
            6)
                full_deploy
                break
                ;;
            0)
                echo "退出"
                break
                ;;
            *)
                echo -e "${RED}✗${NC} 无效选项，请重试"
                echo ""
                ;;
        esac
    done
}

# ============================================================
# 启动主菜单
# ============================================================
main

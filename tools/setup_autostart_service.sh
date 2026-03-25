#!/bin/bash

##############################################################################
# setup_autostart_service.sh
#
# 配置 ROS 2 Vision 系统自启动服务（图形界面环境）
# 支持两种启动方式：
#   1. systemd user service（推荐，完整日志记录）
#   2. Desktop autostart（简单快速）
#
# 用法：
#   bash tools/setup_autostart_service.sh [--method systemd|desktop] [--robot-type default]
#
# 示例：
#   bash tools/setup_autostart_service.sh                    # 交互式菜单
#   bash tools/setup_autostart_service.sh --method systemd   # 使用 systemd 方式
#   bash tools/setup_autostart_service.sh --method desktop   # 使用 desktop 方式
#
##############################################################################

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 脚本路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$HOME/.ckyf_vision_autostart"

# 默认配置
METHOD="systemd"
ROBOT_TYPE="default"
DISPLAY_TERMINAL="xterm"  # 可选: xterm, gnome-terminal, konsole

# ============================================================================
# 函数定义
# ============================================================================

print_header() {
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ 错误: $1${NC}"
}

print_warn() {
    echo -e "${YELLOW}⚠ 警告: $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# 交互式选择方法
select_method() {
    echo ""
    echo "选择启动方式："
    echo "  [1] systemd user service (推荐) - 完整日志记录、可靠启动"
    echo "  [2] Desktop autostart - 简单快速、仅需桌面环境"
    echo ""
    read -p "请选择 [1-2] (默认: 1): " choice
    choice=${choice:-1}

    case $choice in
        1) METHOD="systemd" ;;
        2) METHOD="desktop" ;;
        *) print_error "无效选择"; return 1 ;;
    esac
}

# 交互式选择机型
select_robot_type() {
    echo ""
    echo "选择机器人类型："
    echo "  [1] default"
    echo "  [2] infantry_3"
    echo "  [3] infantry_4"
    echo "  [4] sentry"
    echo ""
    read -p "请选择 [1-4] (默认: 1): " choice
    choice=${choice:-1}

    case $choice in
        1) ROBOT_TYPE="default" ;;
        2) ROBOT_TYPE="infantry_3" ;;
        3) ROBOT_TYPE="infantry_4" ;;
        4) ROBOT_TYPE="sentry" ;;
        *) print_error "无效选择"; return 1 ;;
    esac
}

# 检查终端工具可用性
check_terminal() {
    local terminals=("gnome-terminal" "xterm" "konsole" "xfce4-terminal")

    for term in "${terminals[@]}"; do
        if command -v "$term" &> /dev/null; then
            DISPLAY_TERMINAL="$term"
            print_success "检测到终端: $DISPLAY_TERMINAL"
            return 0
        fi
    done

    print_warn "未检测到图形终端，自启动时将后台运行"
    DISPLAY_TERMINAL=""
}

# ============================================================================
# 方式 1: systemd user service
# ============================================================================

setup_systemd_service() {
    print_header "配置 systemd User Service"

    local service_dir="$HOME/.config/systemd/user"
    local service_file="$service_dir/ckyf-vision.service"
    local wrapper_script="$HOME/.local/bin/ckyf-vision-start.sh"

    # 创建目录
    mkdir -p "$service_dir"
    mkdir -p "$(dirname "$wrapper_script")"
    mkdir -p "$LOG_DIR"

    print_info "创建启动包装脚本: $wrapper_script"

    # 创建包装脚本
    cat > "$wrapper_script" << 'EOF'
#!/bin/bash
# ROS 2 Vision 系统启动脚本
# 此脚本在前台运行，日志输出到终端和文件

set -e

PROJECT_ROOT="/home/mijiao/ckyf_vision"
LOG_DIR="$HOME/.ckyf_vision_autostart"
LOG_FILE="$LOG_DIR/vision.log"
ROBOT_TYPE="${ROBOT_TYPE:-default}"

# 确保日志目录存在
mkdir -p "$LOG_DIR"

# 输出启动信息
{
    echo "====================================================================="
    echo "ckyf_vision ROS 2 系统启动"
    echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "机型: $ROBOT_TYPE"
    echo "项目: $PROJECT_ROOT"
    echo "====================================================================="
    echo ""
} | tee -a "$LOG_FILE"

# 执行启动
cd "$PROJECT_ROOT"
source install/setup.bash

echo "启动 ROS 2 Vision 系统..." | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# 启动系统（日志同时输出到文件和终端）
ros2 launch robot_bringup vision.launch.py robot_type:="$ROBOT_TYPE" 2>&1 | tee -a "$LOG_FILE"
EOF

    chmod +x "$wrapper_script"
    print_success "启动脚本已创建: $wrapper_script"

    # 创建 systemd service 文件
    print_info "创建 systemd 服务文件: $service_file"

    cat > "$service_file" << EOF
[Unit]
Description=ckyf_vision ROS 2 Vision Auto-Aim System
After=network.target graphical-session.target
PartOf=graphical-session.target

[Service]
Type=simple
ExecStart=$wrapper_script
Restart=on-failure
RestartSec=5
Environment="ROBOT_TYPE=$ROBOT_TYPE"
StandardOutput=inherit
StandardError=inherit

[Install]
WantedBy=graphical-session.target
EOF

    print_success "systemd 服务文件已创建: $service_file"

    # 启用服务
    print_info "启用服务..."
    systemctl --user daemon-reload
    systemctl --user enable ckyf-vision.service

    echo ""
    print_success "systemd 服务配置完成！"
    echo ""
    echo "启动方式："
    echo "  • 立即启动:   systemctl --user start ckyf-vision"
    echo "  • 停止服务:   systemctl --user stop ckyf-vision"
    echo "  • 查看状态:   systemctl --user status ckyf-vision"
    echo "  • 查看日志:   journalctl --user -u ckyf-vision -f"
    echo "  • 禁用自启:   systemctl --user disable ckyf-vision"
    echo ""
    echo "日志位置: $LOG_FILE"
}

# ============================================================================
# 方式 2: Desktop autostart
# ============================================================================

setup_desktop_autostart() {
    print_header "配置 Desktop Autostart"

    local autostart_dir="$HOME/.config/autostart"
    local desktop_file="$autostart_dir/ckyf-vision.desktop"
    local wrapper_script="$HOME/.local/bin/ckyf-vision-start.sh"

    # 创建目录
    mkdir -p "$autostart_dir"
    mkdir -p "$(dirname "$wrapper_script")"
    mkdir -p "$LOG_DIR"

    check_terminal

    print_info "创建启动包装脚本: $wrapper_script"

    # 创建包装脚本（用于 desktop autostart）
    if [ -n "$DISPLAY_TERMINAL" ]; then
        cat > "$wrapper_script" << EOF
#!/bin/bash
# ROS 2 Vision 系统启动脚本（Desktop Autostart）

PROJECT_ROOT="/home/mijiao/ckyf_vision"
LOG_DIR="\$HOME/.ckyf_vision_autostart"
LOG_FILE="\$LOG_DIR/vision.log"
ROBOT_TYPE="\${ROBOT_TYPE:-$ROBOT_TYPE}"

mkdir -p "\$LOG_DIR"

# 记录启动信息
{
    echo "====================================================================="
    echo "ckyf_vision ROS 2 系统启动 (Desktop Autostart)"
    echo "时间: \$(date '+%Y-%m-%d %H:%M:%S')"
    echo "机型: \$ROBOT_TYPE"
    echo "项目: \$PROJECT_ROOT"
    echo "====================================================================="
    echo ""
} >> "\$LOG_FILE"

cd "\$PROJECT_ROOT"
source install/setup.bash

ros2 launch robot_bringup vision.launch.py robot_type:="\$ROBOT_TYPE" 2>&1 | tee -a "\$LOG_FILE"
EOF
    else
        cat > "$wrapper_script" << EOF
#!/bin/bash
# ROS 2 Vision 系统启动脚本（后台）

PROJECT_ROOT="/home/mijiao/ckyf_vision"
LOG_DIR="\$HOME/.ckyf_vision_autostart"
LOG_FILE="\$LOG_DIR/vision.log"
ROBOT_TYPE="\${ROBOT_TYPE:-$ROBOT_TYPE}"

mkdir -p "\$LOG_DIR"

cd "\$PROJECT_ROOT"
source install/setup.bash

ros2 launch robot_bringup vision.launch.py robot_type:="\$ROBOT_TYPE" >> "\$LOG_FILE" 2>&1 &
EOF
    fi

    chmod +x "$wrapper_script"
    print_success "启动脚本已创建: $wrapper_script"

    # 创建 .desktop 文件
    print_info "创建 autostart .desktop 文件: $desktop_file"

    if [ -n "$DISPLAY_TERMINAL" ]; then
        case "$DISPLAY_TERMINAL" in
            gnome-terminal)
                cat > "$desktop_file" << EOF
[Desktop Entry]
Type=Application
Name=ckyf_vision ROS 2
Comment=ROS 2 Vision Auto-Aim System Startup
Exec=gnome-terminal -- bash $wrapper_script
Icon=utilities-terminal
Terminal=false
Categories=Development;
X-GNOME-Autostart-enabled=true
EOF
                ;;
            xterm)
                cat > "$desktop_file" << EOF
[Desktop Entry]
Type=Application
Name=ckyf_vision ROS 2
Comment=ROS 2 Vision Auto-Aim System Startup
Exec=xterm -e "bash $wrapper_script; read -p 'Press Enter to close...'"
Icon=utilities-terminal
Terminal=false
Categories=Development;
X-GNOME-Autostart-enabled=true
EOF
                ;;
            konsole)
                cat > "$desktop_file" << EOF
[Desktop Entry]
Type=Application
Name=ckyf_vision ROS 2
Comment=ROS 2 Vision Auto-Aim System Startup
Exec=konsole -e bash $wrapper_script
Icon=utilities-terminal
Terminal=false
Categories=Development;
X-GNOME-Autostart-enabled=true
EOF
                ;;
            *)
                cat > "$desktop_file" << EOF
[Desktop Entry]
Type=Application
Name=ckyf_vision ROS 2
Comment=ROS 2 Vision Auto-Aim System Startup
Exec=$wrapper_script
Icon=utilities-terminal
Terminal=true
Categories=Development;
X-GNOME-Autostart-enabled=true
EOF
                ;;
        esac
    else
        cat > "$desktop_file" << EOF
[Desktop Entry]
Type=Application
Name=ckyf_vision ROS 2
Comment=ROS 2 Vision Auto-Aim System Startup (Background)
Exec=$wrapper_script
Icon=utilities-terminal
Terminal=false
Categories=Development;
X-GNOME-Autostart-enabled=true
Hidden=false
NoDisplay=false
EOF
    fi

    chmod 644 "$desktop_file"
    print_success ".desktop 文件已创建: $desktop_file"

    echo ""
    print_success "Desktop Autostart 配置完成！"
    echo ""
    echo "启动方式："
    echo "  • 下次登录时自动启动"
    echo "  • 若要立即启动，执行: bash $wrapper_script"
    echo ""
    echo "禁用自启动："
    if command -v gnome-control-center &> /dev/null; then
        echo "  • 打开 '设置' > '应用' > '启动应用程序'，禁用 'ckyf_vision ROS 2'"
    fi
    echo "  • 或直接删除: rm $desktop_file"
    echo ""
    echo "日志位置: $LOG_DIR/vision.log"
}

# ============================================================================
# 卸载自启动
# ============================================================================

uninstall_autostart() {
    print_header "卸载自启动配置"

    echo "检测到的自启动配置："

    local found=0
    local systemd_service="$HOME/.config/systemd/user/ckyf-vision.service"
    local desktop_file="$HOME/.config/autostart/ckyf-vision.desktop"

    if [ -f "$systemd_service" ]; then
        echo "  [1] systemd service: $systemd_service"
        found=$((found + 1))
    fi

    if [ -f "$desktop_file" ]; then
        echo "  [2] Desktop autostart: $desktop_file"
        found=$((found + 1))
    fi

    if [ $found -eq 0 ]; then
        print_warn "未找到已安装的自启动配置"
        return 0
    fi

    read -p "确定要卸载吗? (y/n): " confirm
    [ "$confirm" != "y" ] && return 0

    # 卸载 systemd 服务
    if [ -f "$systemd_service" ]; then
        print_info "卸载 systemd 服务..."
        systemctl --user disable ckyf-vision.service 2>/dev/null || true
        systemctl --user stop ckyf-vision.service 2>/dev/null || true
        rm -f "$systemd_service"
        systemctl --user daemon-reload
        print_success "systemd 服务已卸载"
    fi

    # 删除 desktop 文件
    if [ -f "$desktop_file" ]; then
        print_info "删除 autostart 文件..."
        rm -f "$desktop_file"
        print_success "autostart 文件已删除"
    fi

    # 删除启动脚本
    local wrapper_script="$HOME/.local/bin/ckyf-vision-start.sh"
    if [ -f "$wrapper_script" ]; then
        rm -f "$wrapper_script"
        print_success "启动脚本已删除"
    fi

    print_success "自启动配置已完全卸载"
}

# ============================================================================
# 检查状态
# ============================================================================

check_autostart_status() {
    print_header "自启动配置状态"

    local systemd_service="$HOME/.config/systemd/user/ckyf-vision.service"
    local desktop_file="$HOME/.config/autostart/ckyf-vision.desktop"
    local log_file="$HOME/.ckyf_vision_autostart/vision.log"

    echo ""
    echo "【systemd Service】"
    if [ -f "$systemd_service" ]; then
        print_success "配置文件存在: $systemd_service"
        echo ""
        systemctl --user is-enabled ckyf-vision.service &>/dev/null && print_success "自启动已启用" || print_warn "自启动已禁用"
        echo ""
        echo "服务状态:"
        systemctl --user status ckyf-vision.service --no-pager || true
    else
        print_warn "未配置 systemd 服务"
    fi

    echo ""
    echo "【Desktop Autostart】"
    if [ -f "$desktop_file" ]; then
        print_success "配置文件存在: $desktop_file"
    else
        print_warn "未配置 Desktop autostart"
    fi

    echo ""
    echo "【日志文件】"
    if [ -f "$log_file" ]; then
        print_success "日志存在: $log_file"
        echo ""
        echo "最近 10 行日志:"
        tail -10 "$log_file"
    else
        print_warn "暂无日志文件"
    fi
}

# ============================================================================
# 主菜单
# ============================================================================

main_menu() {
    while true; do
        echo ""
        print_header "ROS 2 Vision 系统自启动配置工具"
        echo ""
        echo "请选择操作:"
        echo "  [1] 设置自启动 (交互式)"
        echo "  [2] 使用 systemd 方式"
        echo "  [3] 使用 Desktop autostart 方式"
        echo "  [4] 查看自启动状态"
        echo "  [5] 卸载自启动"
        echo "  [0] 退出"
        echo ""
        read -p "请选择 [0-5]: " choice

        case $choice in
            1)
                select_method || continue
                select_robot_type || continue
                if [ "$METHOD" = "systemd" ]; then
                    setup_systemd_service
                else
                    setup_desktop_autostart
                fi
                ;;
            2)
                METHOD="systemd"
                select_robot_type || continue
                setup_systemd_service
                ;;
            3)
                METHOD="desktop"
                select_robot_type || continue
                setup_desktop_autostart
                ;;
            4)
                check_autostart_status
                ;;
            5)
                uninstall_autostart
                ;;
            0)
                print_success "退出"
                exit 0
                ;;
            *)
                print_error "无效选择"
                ;;
        esac

        read -p "按 Enter 继续..."
    done
}

# ============================================================================
# 命令行参数处理
# ============================================================================

show_help() {
    cat << EOF
用法: $0 [选项]

选项:
  --method SYSTEMD|DESKTOP    指定启动方式 (默认: systemd)
  --robot-type TYPE           指定机型 (default|infantry_3|infantry_4|sentry, 默认: default)
  --status                    查看自启动状态
  --uninstall                 卸载自启动
  --help                      显示此帮助信息

示例:
  bash $0                                      # 交互式菜单
  bash $0 --method systemd --robot-type default
  bash $0 --method desktop --robot-type infantry_4
  bash $0 --status
  bash $0 --uninstall

EOF
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --method)
            METHOD="$2"
            shift 2
            ;;
        --robot-type)
            ROBOT_TYPE="$2"
            shift 2
            ;;
        --status)
            check_autostart_status
            exit 0
            ;;
        --uninstall)
            uninstall_autostart
            exit 0
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

# ============================================================================
# 入口
# ============================================================================

if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    # 检查环境
    if [ ! -d "$PROJECT_ROOT/install" ]; then
        print_error "项目未编译，请先运行: colcon build"
        exit 1
    fi

    # 如果没有指定参数，进入交互菜单
    if [ $# -eq 0 ] || [[ "$*" == *"--method"* ]]; then
        if [ $# -eq 0 ]; then
            main_menu
        else
            # 命令行模式：设置自启动并退出
            if [ "$METHOD" = "systemd" ]; then
                setup_systemd_service
            else
                setup_desktop_autostart
            fi
        fi
    fi
fi

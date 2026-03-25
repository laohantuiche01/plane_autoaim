#!/bin/bash

##############################################################################
# quick_start_vision.sh
#
# 快速启动 ROS 2 Vision 系统（前台运行，便于观察输出）
# 可用于手动前台测试或被自启动服务调用
#
# 用法：
#   bash tools/quick_start_vision.sh [robot_type] [--log-file PATH]
#
# 示例：
#   bash tools/quick_start_vision.sh                    # 使用默认机型
#   bash tools/quick_start_vision.sh infantry_4         # 指定机型
#   bash tools/quick_start_vision.sh default --log-file /tmp/vision.log
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

# 默认配置
ROBOT_TYPE="${1:-default}"
LOG_FILE="${LOG_DIR:-$HOME/.ckyf_vision_autostart}/vision.log"

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --log-file)
            LOG_FILE="$2"
            shift 2
            ;;
        --help)
            cat << EOF
快速启动 ROS 2 Vision 系统

用法: bash $0 [ROBOT_TYPE] [选项]

参数:
  ROBOT_TYPE              机器人类型 (default|infantry_3|infantry_4|sentry)

选项:
  --log-file PATH         日志文件路径 (默认: ~/.ckyf_vision_autostart/vision.log)
  --help                  显示此帮助信息

示例:
  bash $0                           # 使用默认配置
  bash $0 infantry_4                # 指定机型
  bash $0 default --log-file /tmp/vision.log

EOF
            exit 0
            ;;
        *)
            # 第一个非选项参数为 ROBOT_TYPE
            if [[ ! "$1" =~ ^-- ]]; then
                ROBOT_TYPE="$1"
            fi
            shift
            ;;
    esac
done

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
    echo -e "${RED}✗ 错误: $1${NC}" >&2
}

print_warn() {
    echo -e "${YELLOW}⚠ 警告: $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# 检查项目状态
check_project() {
    if [ ! -d "$PROJECT_ROOT/install" ]; then
        print_error "项目未编译，请先执行:"
        echo "  cd $PROJECT_ROOT"
        echo "  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
        exit 1
    fi

    if [ ! -f "$PROJECT_ROOT/install/setup.bash" ]; then
        print_error "找不到项目 setup.bash"
        exit 1
    fi
}

# 检查机型配置
check_robot_type() {
    local config_file="$PROJECT_ROOT/src/robot_bringup/config/${ROBOT_TYPE}/params.yaml"

    if [ ! -f "$config_file" ]; then
        print_error "机型 '$ROBOT_TYPE' 不存在"
        echo ""
        echo "可用的机型:"
        ls "$PROJECT_ROOT/src/robot_bringup/config/" | grep -E "^(default|infantry|sentry)" | sed 's/^/  • /'
        exit 1
    fi
}

# 清理日志（保留最近 10 个）
rotate_logs() {
    local log_dir="$(dirname "$LOG_FILE")"

    if [ -d "$log_dir" ]; then
        local count=$(ls -1 "${log_dir}/vision.log"* 2>/dev/null | wc -l)
        if [ $count -gt 10 ]; then
            print_info "清理旧日志..."
            ls -1t "${log_dir}/vision.log"* 2>/dev/null | tail -n +11 | xargs rm -f
        fi
    fi
}

# 创建启动脚本输出
print_startup_header() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')

    {
        echo ""
        echo "╔═════════════════════════════════════════════════════════════════╗"
        echo "║                  ckyf_vision ROS 2 系统启动                      ║"
        echo "╚═════════════════════════════════════════════════════════════════╝"
        echo ""
        echo "启动时间: $timestamp"
        echo "机    型: $ROBOT_TYPE"
        echo "项目路径: $PROJECT_ROOT"
        echo "日志文件: $LOG_FILE"
        echo ""
        echo "─────────────────────────────────────────────────────────────────"
        echo ""
    } | tee "$LOG_FILE"
}

# 主启动流程
main() {
    print_header "ROS 2 Vision 系统快速启动"

    # 环境检查
    print_info "检查项目环境..."
    check_project
    check_robot_type
    print_success "项目环境检查通过"

    # 日志准备
    local log_dir="$(dirname "$LOG_FILE")"
    mkdir -p "$log_dir"
    rotate_logs

    echo ""
    print_info "配置参数:"
    echo "  • 机    型: $ROBOT_TYPE"
    echo "  • 日志文件: $LOG_FILE"
    echo ""

    # 加载环境
    print_info "加载 ROS 2 环境..."
    cd "$PROJECT_ROOT"
    source install/setup.bash
    print_success "环境加载完成"

    # 打印启动头
    echo ""
    print_startup_header

    # 执行启动（同时输出到终端和日志文件）
    print_info "正在启动 vision.launch.py..."
    echo ""

    # 使用 tee 同时输出到终端和日志
    ros2 launch robot_bringup vision.launch.py robot_type:="$ROBOT_TYPE" 2>&1 | tee -a "$LOG_FILE"

    # 启动失败时的处理
    local exit_code=$?
    if [ $exit_code -ne 0 ]; then
        echo ""
        print_error "系统启动失败 (Exit Code: $exit_code)"
        echo ""
        print_info "故障排除步骤:"
        echo "  1. 检查日志: less $LOG_FILE"
        echo "  2. 验证硬件连接（相机、串口）"
        echo "  3. 检查 ROS 2 环境: ros2 doctor"
        echo "  4. 查看详细信息: ros2 launch robot_bringup vision.launch.py robot_type:=$ROBOT_TYPE debug:=true"
        exit $exit_code
    fi
}

# ============================================================================
# 入口
# ============================================================================

if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi

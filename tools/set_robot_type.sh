#!/bin/bash
# 自动检测并设置 ROBOT_TYPE 环境变量
# 用法：source tools/set_robot_type.sh
# 或在 launch 中调用：bash tools/set_robot_type.sh --export

# 获取主机名
get_robot_type_from_hostname() {
    local hostname=$(hostname -s | tr '[:upper:]' '[:lower:]')

    # 机型识别规则
    case "$hostname" in
        *infantry-3*) echo "infantry_3" ;;
        *infantry-4*) echo "infantry_4" ;;
        *infantry_3*) echo "infantry_3" ;;
        *infantry_4*) echo "infantry_4" ;;
        *sentry*) echo "sentry" ;;
        *outpost*) echo "outpost" ;;
        *)
            # 如果无法识别，使用主机名作为类型
            echo "$hostname"
            ;;
    esac
}

# 主函数
if [[ "$1" == "--export" ]] || [[ "$1" == "--get" ]]; then
    # 非交互模式（用于 launch 或脚本调用）
    get_robot_type_from_hostname
else
    # 交互模式或 source 调用
    local detected_type=$(get_robot_type_from_hostname)

    if [ -z "$ROBOT_TYPE" ]; then
        export ROBOT_TYPE="$detected_type"
        echo "✅ ROBOT_TYPE set to: $ROBOT_TYPE"
    else
        echo "ℹ️  ROBOT_TYPE already set to: $ROBOT_TYPE"
    fi

    # 如果在 shell 中 source 调用，保持 ROBOT_TYPE 环境变量
    if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
        # 已通过 source 调用，环境变量已设置
        :
    fi
fi

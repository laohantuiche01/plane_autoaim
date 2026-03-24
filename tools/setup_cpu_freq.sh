#!/bin/bash
# CPU 频率调控器快速配置脚本
# 用法：sudo bash tools/setup_cpu_freq.sh
# 功能：将 CPU 频率调控器从 powersave 设置为 performance（需要 root）

if [ "$EUID" -ne 0 ]; then
    echo "❌ 请以 root 权限运行：sudo bash $0"
    exit 1
fi

echo "======================================================================"
echo "   CPU 频率调控器配置"
echo "======================================================================"
echo ""

# 检查 cpupower 命令
if ! command -v cpupower &>/dev/null; then
    echo "⚠️  cpupower 未安装，尝试安装..."
    apt-get update -qq && apt-get install -y cpupower >/dev/null 2>&1

    if ! command -v cpupower &>/dev/null; then
        echo "❌ cpupower 安装失败"
        exit 1
    fi
fi

# 获取 CPU 核心数
NUM_CPUS=$(nproc)
echo "检测到 $NUM_CPUS 个 CPU 核心"
echo ""

# 设置所有核心为 performance
echo "正在设置 CPU 频率调控器为 performance..."
for ((i=0; i<NUM_CPUS; i++)); do
    if cpupower -c $i frequency-set -g performance 2>/dev/null; then
        freq=$(cat /sys/devices/system/cpu/cpu${i}/cpufreq/scaling_max_freq 2>/dev/null)
        echo "  ✅ CPU $i: performance (max freq: $((freq/1000)) MHz)"
    else
        # 如果 cpupower 失败，尝试直接写入
        if echo "performance" > /sys/devices/system/cpu/cpu${i}/cpufreq/scaling_governor 2>/dev/null; then
            echo "  ✅ CPU $i: performance (direct write)"
        else
            echo "  ❌ CPU $i: 设置失败"
        fi
    fi
done

echo ""
echo "======================================================================"
echo "  ✅ CPU 频率调控器已设置为 performance"
echo "======================================================================"
echo ""

# 验证
echo "验证当前状态："
for ((i=0; i<NUM_CPUS && i<4; i++)); do
    gov=$(cat /sys/devices/system/cpu/cpu${i}/cpufreq/scaling_governor 2>/dev/null || echo "N/A")
    freq=$(cat /sys/devices/system/cpu/cpu${i}/cpufreq/scaling_cur_freq 2>/dev/null || echo "0")
    echo "  CPU $i: governor=$gov, freq=$((freq/1000)) MHz"
done

if [ "$NUM_CPUS" -gt 4 ]; then
    echo "  ... (其余 $((NUM_CPUS-4)) 个核心类似)"
fi

echo ""
echo "💡 提示：此配置立即生效，无需重启"
echo "   如需永久保存，请在 /etc/rc.local 或系统启动脚本中调用此脚本"
echo ""

#!/bin/bash
# RT 性能状态检验脚本
# 用法：bash tools/check_rt_status.sh [--bench]
# 功能：全面检测 RT 配置状态，生成详细报告
#       --bench 选项额外运行基准测试（需要vision系统正在运行）

PASS="✅ [PASS]"
WARN="⚠️  [WARN]"
FAIL="❌ [FAIL]"

# 获取vision进程ID
VISION_PROC=$(pgrep -x "component_container_isolated" | head -1)

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo ""
echo -e "${GREEN}======================================================================"
echo "   RT 状态检查报告 - $(date '+%Y-%m-%d %H:%M:%S')"
echo "======================================================================${NC}"
echo ""

# ============================================================
# 1. 内核隔离参数检查
# ============================================================
echo -e "${YELLOW}[1] 内核隔离参数${NC}"
echo "────────────────────────────────────────────────────────────────"

if grep -q "isolcpus" /proc/cmdline; then
    ISOLCPUS=$(grep -o 'isolcpus=[^ ]*' /proc/cmdline)
    echo "$PASS isolcpus: $ISOLCPUS"
else
    echo "$FAIL isolcpus 未设置（需修改 /etc/default/grub 并 reboot）"
fi

if grep -q "nohz_full" /proc/cmdline; then
    NOHZ=$(grep -o 'nohz_full=[^ ]*' /proc/cmdline)
    echo "$PASS nohz_full: $NOHZ"
else
    echo "$WARN nohz_full 未设置（可选，用于禁用时钟中断）"
fi

if grep -q "rcu_nocbs" /proc/cmdline; then
    RCU=$(grep -o 'rcu_nocbs=[^ ]*' /proc/cmdline)
    echo "$PASS rcu_nocbs: $RCU"
else
    echo "$WARN rcu_nocbs 未设置（可选，用于卸载 RCU callback）"
fi

# ============================================================
# 2. 调度策略检查
# ============================================================
echo ""
echo -e "${YELLOW}[2] 视觉进程调度策略${NC}"
echo "────────────────────────────────────────────────────────────────"

if [ -z "$VISION_PROC" ]; then
    echo "$WARN component_container_isolated 未运行（启动vision系统后重试）"
else
    SCHED_INFO=$(chrt -p "$VISION_PROC" 2>&1)
    if echo "$SCHED_INFO" | grep -q "SCHED_FIFO"; then
        PRIO=$(echo "$SCHED_INFO" | grep -o "priority [0-9]*" | awk '{print $2}')
        echo "$PASS $SCHED_INFO"
        if [ "$PRIO" -ge 90 ]; then
            echo "$PASS 优先级: $PRIO（建议≥90）"
        else
            echo "$WARN 优先级: $PRIO（建议≥90）"
        fi
    elif echo "$SCHED_INFO" | grep -q "SCHED_OTHER"; then
        echo "$FAIL 调度策略: $SCHED_INFO（应为 SCHED_FIFO）"
    else
        echo "$WARN 调度策略: $SCHED_INFO"
    fi
fi

# ============================================================
# 3. CPU 绑核检查
# ============================================================
echo ""
echo -e "${YELLOW}[3] CPU 亲和性${NC}"
echo "────────────────────────────────────────────────────────────────"

if [ -n "$VISION_PROC" ]; then
    AFFINITY=$(taskset -cp "$VISION_PROC" 2>&1)
    echo "  $AFFINITY"

    if echo "$AFFINITY" | grep -qE "0,1,2,3|0-3"; then
        echo "$PASS CPU 绑核正确（core 0-3）"
    else
        echo "$WARN CPU 绑核可能异常"
    fi
else
    echo "$WARN vision 进程未运行，无法检查 CPU 亲和性"
fi

# ============================================================
# 4. CPU 频率策略
# ============================================================
echo ""
echo -e "${YELLOW}[4] CPU 频率策略${NC}"
echo "────────────────────────────────────────────────────────────────"

for cpu in 0 1 2 3; do
    GOV=$(cat /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
    FREQ=$(cat /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_cur_freq 2>/dev/null || echo "0")

    if [ "$GOV" = "performance" ]; then
        echo "$PASS core${cpu}: ${GOV} @ $((FREQ/1000))MHz"
    else
        echo "$WARN core${cpu}: ${GOV} @ $((FREQ/1000))MHz（建议 performance）"
    fi
done

# ============================================================
# 5. 内存锁定状态
# ============================================================
echo ""
echo -e "${YELLOW}[5] 内存锁定状态${NC}"
echo "────────────────────────────────────────────────────────────────"

if [ -n "$VISION_PROC" ]; then
    VMPIN=$(grep "^VmPin" /proc/"$VISION_PROC"/status 2>/dev/null || echo "VmPin: 0 kB")
    VMRSS=$(grep "^VmRSS" /proc/"$VISION_PROC"/status 2>/dev/null || echo "VmRSS: 0 kB")

    echo "  $VMPIN"
    echo "  $VMRSS"

    # 判断是否启用了mlockall
    PINNED=$(echo "$VMPIN" | awk '{print $2}')
    RESIDENT=$(echo "$VMRSS" | awk '{print $2}')

    if [ "$PINNED" -gt 0 ]; then
        RATIO=$((PINNED * 100 / (RESIDENT > 0 ? RESIDENT : 1)))
        if [ "$RATIO" -gt 90 ]; then
            echo "$PASS mlockall 已启用（锁定率: ${RATIO}%）"
        else
            echo "$WARN mlockall 部分启用（锁定率: ${RATIO}%）"
        fi
    else
        echo "$WARN mlockall 未启用（VmPin = 0）"
    fi
else
    echo "$WARN vision 进程未运行，无法检查内存锁定"
fi

# ============================================================
# 6. IRQ 亲和性检查
# ============================================================
echo ""
echo -e "${YELLOW}[6] USB IRQ 亲和性${NC}"
echo "────────────────────────────────────────────────────────────────"

IRQ_COUNT=0
MIGRATED_COUNT=0

while IFS= read -r line; do
    irq=$(echo "$line" | awk -F: '{print $1}' | tr -d ' ')
    if [ -n "$irq" ]; then
        IRQ_COUNT=$((IRQ_COUNT+1))
        aff=$(cat /proc/irq/"$irq"/smp_affinity 2>/dev/null || echo "unknown")

        # 检查亲和性是否为0x0f（core 0-3）
        if [ "$aff" = "0f" ] || [ "$aff" = "f" ]; then
            echo "$WARN IRQ $irq 仍在隔离核（亲和性=0x${aff}）"
        else
            MIGRATED_COUNT=$((MIGRATED_COUNT+1))
            echo "$PASS IRQ $irq 已迁出隔离核（亲和性=0x${aff}）"
        fi
    fi
done < <(grep -iE "xhci|usb3|usb2" /proc/interrupts 2>/dev/null | head -10 || true)

if [ $IRQ_COUNT -eq 0 ]; then
    echo "$WARN 未找到 USB IRQ"
else
    echo ""
    echo "  摘要：$MIGRATED_COUNT/$IRQ_COUNT 个 USB IRQ 已迁移出隔离核"
fi

# ============================================================
# 7. 非自愿上下文切换统计
# ============================================================
echo ""
echo -e "${YELLOW}[7] 上下文切换实时统计（5秒采样）${NC}"
echo "────────────────────────────────────────────────────────────────"

if [ -z "$VISION_PROC" ]; then
    echo "$WARN vision 进程未运行，跳过统计"
elif command -v pidstat &>/dev/null; then
    echo "  cswch/s    = 自愿上下文切换（比率低更好）"
    echo "  nvcswch/s  = 非自愿上下文切换（目标: < 2）"
    echo ""
    pidstat -w -p "$VISION_PROC" 1 5 2>/dev/null | tail -7
else
    echo "$WARN pidstat 未安装（运行：sudo bash tools/setup_rt_full.sh --install-tools）"
fi

# ============================================================
# 8. limits.conf 权限检查
# ============================================================
echo ""
echo -e "${YELLOW}[8] 用户权限配置${NC}"
echo "────────────────────────────────────────────────────────────────"

USER=$(whoami)
LIMITS_FILE="/etc/security/limits.conf"

for perm in nice rtprio memlock; do
    if grep -q "^$USER.*$perm" "$LIMITS_FILE" 2>/dev/null; then
        VAL=$(grep "^$USER.*$perm" "$LIMITS_FILE" | head -1 | awk '{print $NF}')
        echo "$PASS $perm: $VAL（用户 $USER）"
    else
        echo "$WARN $perm: 未配置（用户 $USER）"
    fi
done

# ============================================================
# 9. 可选基准测试
# ============================================================
if [ "$1" = "--bench" ]; then
    echo ""
    echo -e "${YELLOW}[BENCH] RT 延迟基准测试${NC}"
    echo "────────────────────────────────────────────────────────────────"

    if command -v cyclictest &>/dev/null; then
        echo ""
        echo "  运行 cyclictest（10秒，采样间隔 1ms）..."
        echo "  （这是衡量 RT 系统延迟的黄金标准）"
        echo ""

        # 在隔离核0上运行，FIFO priority 90，采样10000次
        OUTPUT=$(cyclictest -l 10000 -m -S -p 90 -i 1000 -a 0 --quiet 2>&1 || true)
        echo "$OUTPUT" | grep "T: 0" | awk '{
            printf "  Min:    %5s μs\n", $4
            printf "  Avg:    %5s μs\n", $7
            printf "  Max:    %5s μs\n", $10
        }' || echo "  ⚠️  无法解析 cyclictest 输出"

        echo ""
        echo "  📊 评估标准："
        echo "     Max < 50μs   : 优秀（超低延迟）"
        echo "     Max < 200μs  : 良好（RT可用）"
        echo "     Max > 500μs  : 需要优化"
    else
        echo "$WARN cyclictest 未安装（运行：sudo bash tools/setup_rt_full.sh --install-tools）"
    fi

    # perf cache 统计
    echo ""
    echo -e "${YELLOW}[BENCH] perf 缓存统计${NC}"
    echo "────────────────────────────────────────────────────────────────"

    if [ -n "$VISION_PROC" ] && command -v perf &>/dev/null; then
        echo ""
        echo "  采样 10 秒..."
        perf stat -e cache-misses,cache-references,context-switches,page-faults \
            -p "$VISION_PROC" -- sleep 10 2>&1 | grep -E "cache|context|page|#" || true
    else
        echo "$WARN perf 未安装或 vision 进程未运行"
    fi
fi

# ============================================================
# 总结
# ============================================================
echo ""
echo -e "${GREEN}======================================================================"
echo "   检查完成"
echo "======================================================================${NC}"
echo ""
echo "📌 下一步建议："
echo ""
echo "  • 如果有 [FAIL] 项：运行 sudo bash tools/setup_rt_full.sh 后 reboot"
echo "  • 如果所有项都 [PASS]：运行 --bench 进行基准测试"
echo "  • 查看详细日志：journalctl -xe"
echo ""

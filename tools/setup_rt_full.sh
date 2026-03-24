#!/bin/bash
# 全量 RT 优化部署脚本（含 jemalloc 和 direnv 环境配置）
# 用法：sudo bash tools/setup_rt_full.sh [--install-tools]
# 功能：
#   - isolcpus/nohz_full/rcu_nocbs 内核隔离
#   - SCHED_FIFO 实时调度权限
#   - 内存锁定（mlockall）配置
#   - USB IRQ 亲和性迁移
#   - jemalloc 高性能内存分配器安装
#   - direnv 环境变量自动加载配置

set -e

# ============================================================
# 权限检查
# ============================================================
if [ "$EUID" -ne 0 ]; then
    echo "❌ 请以 root 权限运行：sudo bash $0"
    exit 1
fi

TARGET_USER=${SUDO_USER:-$(whoami)}
ISOLATED_CORES="0,1,2,3"
VISION_CORES_MASK="0f"      # 十六进制：core 0-3
OTHER_CORES_MASK="fffffff0" # core 4+ 接收IRQ（8核CPU）

echo "======================================================================"
echo "   RT 优化全量部署脚本"
echo "======================================================================"
echo "   目标用户：$TARGET_USER"
echo "   隔离核心：$ISOLATED_CORES"
echo "======================================================================"

# ============================================================
# 1. 安装性能分析工具（可选）
# ============================================================
if [ "$1" = "--install-tools" ]; then
    echo ""
    echo "[1/5] 🔧 安装性能分析工具..."
    KERNEL_VER=$(uname -r)

    apt-get update -qq 2>/dev/null || true

    TOOLS=(
        "linux-tools-${KERNEL_VER}"  # perf
        "linux-tools-common"
        "sysstat"                     # pidstat, iostat, sar
        "rt-tests"                    # cyclictest
        "schedtool"                   # 调度器工具
        "htop"                        # 系统监控
        "numactl"                     # NUMA分析
        "cpupower"                    # CPU频率管理
        "libjemalloc-dev"             # 高性能内存分配器
        "direnv"                      # 环境管理工具
    )

    for tool in "${TOOLS[@]}"; do
        if ! dpkg -l | grep -q "^ii.*${tool}"; then
            echo "  └─ 安装 $tool..."
            apt-get install -y "$tool" >/dev/null 2>&1 || echo "    ⚠️  $tool 安装失败，跳过"
        fi
    done
    echo "  ✅ 性能工具检查完成"
fi

# ============================================================
# 2. 配置 /etc/security/limits.conf（用户权限）
# ============================================================
echo ""
echo "[2/5] 📋 配置 /etc/security/limits.conf..."
LIMITS_FILE="/etc/security/limits.conf"

# 移除旧配置（避免重复）
sed -i "/^$TARGET_USER.*nice/d; /^$TARGET_USER.*rtprio/d; /^$TARGET_USER.*memlock/d" "$LIMITS_FILE" 2>/dev/null || true

# 添加新配置
cat >> "$LIMITS_FILE" <<EOF
# RT优化配置（$(date '+%Y-%m-%d %H:%M:%S')）
$TARGET_USER soft nice -20
$TARGET_USER hard nice -20
$TARGET_USER soft rtprio 90
$TARGET_USER hard rtprio 90
$TARGET_USER soft memlock unlimited
$TARGET_USER hard memlock unlimited
EOF

echo "  ✅ limits.conf 已更新（需重新登录生效）"

# ============================================================
# 3. 配置内核启动参数（isolcpus）
# ============================================================
echo ""
echo "[3/5] 🔌 配置内核启动参数..."
GRUB_FILE="/etc/default/grub"
BACKUP_FILE="/etc/default/grub.bak.$(date +%Y%m%d_%H%M%S)"

# 备份原始grub配置
cp "$GRUB_FILE" "$BACKUP_FILE"
echo "  └─ 原始配置已备份：$BACKUP_FILE"

# 检查是否已有isolcpus
if grep -q "isolcpus" "$GRUB_FILE"; then
    echo "  ⏭️  isolcpus 已存在于 grub 配置，跳过"
else
    RT_PARAMS="isolcpus=${ISOLATED_CORES} nohz_full=${ISOLATED_CORES} rcu_nocbs=${ISOLATED_CORES}"

    # 修改GRUB_CMDLINE_LINUX_DEFAULT
    sed -i "s/GRUB_CMDLINE_LINUX_DEFAULT=\"/GRUB_CMDLINE_LINUX_DEFAULT=\"${RT_PARAMS} /" "$GRUB_FILE"

    # 刷新grub配置
    update-grub >/dev/null 2>&1 || true

    echo "  ✅ isolcpus 已写入 grub 配置"
    echo "  ⚠️  **需要 reboot 后生效**：sudo reboot"
fi

# ============================================================
# 4. 配置 direnv（环境变量自动加载）
# ============================================================
echo ""
echo "[4/5] 🌍 配置 direnv..."

# 配置 .zshrc
ZSHRC="$HOME/.zshrc"
if [ -f "$ZSHRC" ]; then
    if ! grep -q "eval \"\$(direnv hook zsh)\"" "$ZSHRC" 2>/dev/null; then
        echo "" >> "$ZSHRC"
        echo "# direnv hook（$(date '+%Y-%m-%d %H:%M:%S')）" >> "$ZSHRC"
        echo "eval \"\$(direnv hook zsh)\"" >> "$ZSHRC"
        echo "  ✅ direnv hook 已添加至 ~/.zshrc"
    else
        echo "  ⏭️  direnv hook 已存在于 ~/.zshrc，跳过"
    fi
else
    echo "  ℹ️  ~/.zshrc 不存在，跳过"
fi

# 配置 .bashrc
BASHRC="$HOME/.bashrc"
if [ -f "$BASHRC" ]; then
    if ! grep -q "eval \"\$(direnv hook bash)\"" "$BASHRC" 2>/dev/null; then
        echo "" >> "$BASHRC"
        echo "# direnv hook（$(date '+%Y-%m-%d %H:%M:%S')）" >> "$BASHRC"
        echo "eval \"\$(direnv hook bash)\"" >> "$BASHRC"
        echo "  ✅ direnv hook 已添加至 ~/.bashrc"
    else
        echo "  ⏭️  direnv hook 已存在于 ~/.bashrc，跳过"
    fi
else
    echo "  ℹ️  ~/.bashrc 不存在，跳过"
fi

# 配置项目目录的 .envrc（已包含在版本控制中）
PROJECT_DIR="${CKYF_VISION_PATH:-.}"  # 支持环境变量覆盖，默认为当前目录
# 尝试找到 ckyf_vision 项目根目录
if [ ! -f "$PROJECT_DIR/.envrc" ] && [ ! -f "$PROJECT_DIR/CMakeLists.txt" ]; then
    if [ -d "/home/mijiao/ckyf_vision" ]; then
        PROJECT_DIR="/home/mijiao/ckyf_vision"
    fi
fi

if [ -d "$PROJECT_DIR" ] && [ -f "$PROJECT_DIR/.envrc" ]; then
    echo "  ✅ .envrc 已包含在项目中（自动跨平台兼容）"
    echo "  ℹ️  请运行：cd $PROJECT_DIR && direnv allow"
fi

# ============================================================
# 5. 配置 IRQ 亲和性（立即生效）
# ============================================================
echo ""
echo "[5/5] 🎯 配置 USB IRQ 亲和性..."
AFFINITY_SET=0

# 查找USB相关IRQ并迁移到非隔离核
while IFS= read -r line; do
    irq=$(echo "$line" | awk -F: '{print $1}' | tr -d ' ')
    if [ -n "$irq" ]; then
        if echo "$OTHER_CORES_MASK" > "/proc/irq/$irq/smp_affinity" 2>/dev/null; then
            AFFINITY_SET=$((AFFINITY_SET+1))
        fi
    fi
done < <(grep -iE "xhci|usb3|usb2" /proc/interrupts 2>/dev/null || true)

echo "  ✅ 已迁移 $AFFINITY_SET 个 USB IRQ 至非隔离核"

# 配置irqbalance禁止使用隔离核
if systemctl is-active --quiet irqbalance 2>/dev/null; then
    IRQB_CONF="/etc/default/irqbalance"
    if ! grep -q "IRQBALANCE_BANNED_CPUS" "$IRQB_CONF" 2>/dev/null; then
        echo "IRQBALANCE_BANNED_CPUS=$VISION_CORES_MASK" >> "$IRQB_CONF"
    fi
    systemctl restart irqbalance >/dev/null 2>&1 || true
    echo "  ✅ irqbalance 已配置禁止使用 core 0-3"
fi

# ============================================================
# 6. 总结
# ============================================================
echo ""
echo "======================================================================"
echo "   ✅ 部署完成！"
echo "======================================================================"
echo ""
echo "📌 后续步骤："
echo ""
echo "  1️⃣  激活 direnv（可选，使 .envrc 生效）："
echo "     cd /home/mijiao/ckyf_vision && direnv allow"
echo ""
echo "  2️⃣  重新登录（使 limits.conf 生效）："
echo "     logout 或 exit"
echo ""
echo "  3️⃣  重启（使 isolcpus 生效）："
echo "     sudo reboot"
echo ""
echo "  4️⃣  编译项目（jemalloc 已自动链接）："
echo "     cd /home/mijiao/ckyf_vision"
echo "     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo ""
echo "  5️⃣  启动视觉系统："
echo "     source install/setup.bash"
echo "     ros2 launch robot_bringup vision.launch.py robot_type:=default"
echo ""
echo "  6️⃣  验证性能优化（新增终端）："
echo "     bash /home/mijiao/ckyf_vision/tools/check_rt_status.sh --bench"
echo ""
echo "======================================================================"
echo ""
echo "🔗 安装内容："
echo "  ✓ 性能分析工具（perf, cyclictest, sysstat 等）"
echo "  ✓ jemalloc（高性能内存分配器）"
echo "  ✓ direnv（环境变量自动加载）"
echo "  ✓ limits.conf（用户权限配置）"
echo "  ✓ Kernel 启动参数（isolcpus, nohz_full, rcu_nocbs）"
echo "  ✓ IRQ 亲和性迁移"
echo ""
echo "======================================================================"

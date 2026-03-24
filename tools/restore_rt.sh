#!/bin/bash
# RT 优化配置恢复脚本（回滚）
# 用法：sudo bash tools/restore_rt.sh
# 功能：将所有 RT 配置恢复到系统默认状态

set -e

# ============================================================
# 权限检查
# ============================================================
if [ "$EUID" -ne 0 ]; then
    echo "❌ 请以 root 权限运行：sudo bash $0"
    exit 1
fi

TARGET_USER=${SUDO_USER:-$(whoami)}

echo "======================================================================"
echo "   RT 优化配置恢复脚本（回滚）"
echo "======================================================================"
echo "   目标用户：$TARGET_USER"
echo "======================================================================"
echo ""
echo "⚠️  此操作将恢复所有 RT 优化配置到系统默认状态"
read -p "确认继续？(yes/no): " confirm
if [ "$confirm" != "yes" ]; then
    echo "已取消"
    exit 0
fi

# ============================================================
# 1. 恢复 /etc/default/grub
# ============================================================
echo ""
echo "[1/3] 🔌 恢复内核启动参数..."

GRUB_FILE="/etc/default/grub"
LATEST_BACKUP=$(ls -t /etc/default/grub.bak.* 2>/dev/null | head -1)

if [ -n "$LATEST_BACKUP" ]; then
    cp "$LATEST_BACKUP" "$GRUB_FILE"
    update-grub >/dev/null 2>&1 || true
    echo "  ✅ grub 已从备份恢复：$LATEST_BACKUP"
    echo "  ⚠️  **需要 reboot 后生效**"
else
    # 手动删除RT参数
    sed -i 's/ isolcpus=[^ "]*//g; s/ nohz_full=[^ "]*//g; s/ rcu_nocbs=[^ "]*//g' "$GRUB_FILE"
    update-grub >/dev/null 2>&1 || true
    echo "  ✅ grub RT 参数已手动移除"
    echo "  ⚠️  **需要 reboot 后生效**"
fi

# ============================================================
# 2. 恢复 /etc/security/limits.conf
# ============================================================
echo ""
echo "[2/3] 📋 恢复 /etc/security/limits.conf..."

LIMITS_FILE="/etc/security/limits.conf"
sed -i "/^$TARGET_USER.*nice/d; /^$TARGET_USER.*rtprio/d; /^$TARGET_USER.*memlock/d" "$LIMITS_FILE" 2>/dev/null || true
sed -i '/# RT优化配置/d' "$LIMITS_FILE" 2>/dev/null || true

echo "  ✅ limits.conf RT 权限已移除"

# ============================================================
# 3. 恢复 irqbalance
# ============================================================
echo ""
echo "[3/3] 🎯 恢复 irqbalance 配置..."

IRQB_CONF="/etc/default/irqbalance"
if [ -f "$IRQB_CONF" ]; then
    sed -i '/IRQBALANCE_BANNED_CPUS/d' "$IRQB_CONF" 2>/dev/null || true
fi

if systemctl is-active --quiet irqbalance 2>/dev/null; then
    systemctl restart irqbalance >/dev/null 2>&1 || true
fi

echo "  ✅ irqbalance 已恢复默认配置"

# ============================================================
# 总结
# ============================================================
echo ""
echo "======================================================================"
echo "   ✅ 恢复完成！"
echo "======================================================================"
echo ""
echo "📌 后续步骤："
echo ""
echo "  1️⃣  重启使 grub 配置生效："
echo "     sudo reboot"
echo ""
echo "  2️⃣  验证恢复："
echo "     cat /proc/cmdline | grep isolcpus  （应无输出）"
echo ""
echo "======================================================================"

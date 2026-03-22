#!/bin/bash

# 检查是否以root权限运行，因为配置udev规则需要管理员权限
if [ "$EUID" -ne 0 ]; then
  echo "请以 root 权限运行此脚本 (使用 sudo ./bind_usb_serial.sh)"
  exit 1
fi

# 获取当前实际运行 sudo 的用户
if [ -n "$SUDO_USER" ]; then
    TARGET_USER="$SUDO_USER"
else
    TARGET_USER="$USER"
fi

# 1. 检查并添加用户到 dialout 组 (串口设备默认属于该组)
# 注意：通常串口用户组是 dialout，而不是 dialog
GROUP_NAME="dialout"
if id -nG "$TARGET_USER" | grep -qw "$GROUP_NAME"; then
    echo "✅ 用户 $TARGET_USER 已经在 $GROUP_NAME 组中。"
else
    echo "⚙️ 正在将用户 $TARGET_USER 添加到 $GROUP_NAME 组..."
    usermod -aG "$GROUP_NAME" "$TARGET_USER"
    echo "⚠️ 注意: 你可能需要注销并重新登录，用户组的更改才能完全生效。"
fi

echo "------------------------------------------------"
echo "🔍 正在扫描当前的 USB 串口设备..."

devices=()
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        devices+=("$dev")
    fi
done

if [ ${#devices[@]} -eq 0 ]; then
    echo "❌ 未找到任何 USB 串口设备 (/dev/ttyUSB* 或 /dev/ttyACM*)."
    exit 0
fi

echo "以下是检测到的可用设备:"
index=1
declare -A dev_info

for dev in "${devices[@]}"; do
    # 获取设备的 vendor (供应商ID), product (产品ID), 和 serial (序列号)
    vendor=$(udevadm info -a -n "$dev" | grep '{idVendor}' | head -n1 | cut -d'"' -f2)
    product=$(udevadm info -a -n "$dev" | grep '{idProduct}' | head -n1 | cut -d'"' -f2)
    serial=$(udevadm info -a -n "$dev" | grep '{serial}' | head -n1 | cut -d'"' -f2)
    
    dev_info[$index,dev]="$dev"
    dev_info[$index,vendor]="$vendor"
    dev_info[$index,product]="$product"
    dev_info[$index,serial]="$serial"
    
    echo "[$index] $dev"
    echo "    ↳ Vendor:  ${vendor:-未知}"
    echo "    ↳ Product: ${product:-未知}"
    echo "    ↳ Serial:  ${serial:-未知}"
    ((index++))
done

echo "------------------------------------------------"

# 2. 交互式选择
read -p "⌨️  请输入你要绑定的设备序号 (1-$((index-1))): " sel

if ! [[ "$sel" =~ ^[0-9]+$ ]] || [ "$sel" -lt 1 ] || [ "$sel" -ge "$index" ]; then
    echo "❌ 无效的序号。"
    exit 1
fi

selected_dev="${dev_info[$sel,dev]}"
vendor="${dev_info[$sel,vendor]}"
product="${dev_info[$sel,product]}"
serial="${dev_info[$sel,serial]}"

if [ -z "$vendor" ] || [ -z "$product" ]; then
    echo "❌ 无法获取 $selected_dev 的 Vendor 或 Product ID，无法创建可靠的 udev 规则。"
    exit 1
fi

# 3. 输入要绑定的名称
echo "------------------------------------------------"
read -p "⌨️  请输入你想要固定的别名 (例如 ttyRobot, 不需要加 /dev/): " symlink_name

# 去除用户可能错误输入的 /dev/
symlink_name=${symlink_name#/dev/}

if [ -z "$symlink_name" ]; then
    echo "❌ 别名不能为空。"
    exit 1
fi

# 4. 创建 udev 规则
rule_file="/etc/udev/rules.d/99-${symlink_name}.rules"

# 如果有序列号，使用序列号进行精确匹配；否则退化为使用 Vendor ID 和 Product ID 匹配
if [ -n "$serial" ]; then
    rule="KERNEL==\"tty[A-Z]*[0-9]*\", SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$product\", ATTRS{serial}==\"$serial\", SYMLINK+=\"$symlink_name\", MODE=\"0666\""
else
    echo "⚠️ 警告: 该设备没有硬件序列号。规则将仅依赖于 Vendor 和 Product ID 绑定。如果你插入多个同型号的设备，可能会发生冲突。"
    rule="KERNEL==\"tty[A-Z]*[0-9]*\", SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$product\", SYMLINK+=\"$symlink_name\", MODE=\"0666\""
fi

echo "📝 正在生成 udev 规则到 $rule_file ..."
echo "$rule" > "$rule_file"

# 5. 重新加载 udev 规则
echo "🔄 正在重新加载 udev 规则以使其立即生效..."
udevadm control --reload-rules
udevadm trigger

echo "------------------------------------------------"
echo "✅ 配置完成！"
echo "设备现在可以通过 /dev/$symlink_name 访问 (已附加 0666 读写权限)。"
echo "你可以运行命令来检查: ls -l /dev/$symlink_name"

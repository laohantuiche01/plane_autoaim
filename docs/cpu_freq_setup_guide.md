# CPU 频率调控器快速修复指南

## 问题

运行 `check_rt_status.sh` 时显示 CPU 频率调控器为 `powersave` 而不是 `performance`：

```
❌ [WARN] core0: powersave @ 2900MHz（建议 performance）
❌ [WARN] core1: powersave @ 2900MHz（建议 performance）
...
```

这会导致 CPU 无法全速运行，影响实时性能。

## 快速修复（立即生效，无需重启）

### 方式 1：使用专用脚本（推荐）

```bash
# 车上运行
sudo bash tools/setup_cpu_freq.sh
```

**优点**：
- 立即生效
- 自动检测所有 CPU 核心
- 优雅处理缺少 cpupower 的情况

### 方式 2：手动命令（无需脚本）

```bash
# 方法 A：使用 cpupower（若已安装）
sudo apt install cpupower
for i in 0 1 2 3; do
  sudo cpupower -c $i frequency-set -g performance
done

# 方法 B：直接写入 sysfs（不需要 cpupower）
for i in 0 1 2 3; do
  echo "performance" | sudo tee /sys/devices/system/cpu/cpu${i}/cpufreq/scaling_governor
done
```

### 方式 3：通过完整部署脚本

```bash
# 若全量运行 setup_rt_full.sh，已包含此步骤
sudo bash tools/setup_rt_full.sh --install-tools
```

## 验证修复

```bash
# 立即验证
bash tools/check_rt_status.sh

# 应该看到：
✅ [PASS] core0: performance @ 3600MHz
✅ [PASS] core1: performance @ 3600MHz
✅ [PASS] core2: performance @ 3600MHz
✅ [PASS] core3: performance @ 3600MHz
```

## 永久配置（可选）

若需要重启后仍保持 performance：

### Ubuntu 18.04+（使用 systemd）

创建 `/etc/systemd/system/cpu-freq-performance.service`：

```ini
[Unit]
Description=Set CPU frequency to performance
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/bash -c 'for i in {0..3}; do echo performance > /sys/devices/system/cpu/cpu${i}/cpufreq/scaling_governor; done'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

然后启用：

```bash
sudo systemctl daemon-reload
sudo systemctl enable cpu-freq-performance.service
```

### 或使用 /etc/rc.local

```bash
echo '#!/bin/bash' | sudo tee /etc/rc.local
echo 'for i in {0..3}; do echo performance > /sys/devices/system/cpu/cpu${i}/cpufreq/scaling_governor; done' | sudo tee -a /etc/rc.local
sudo chmod +x /etc/rc.local
```

## 原因分析

### 为什么会是 powersave？

- 系统默认频率调控器为 `powersave` 以省电
- `setup_rt_full.sh` 设置 `isolcpus` 内核参数时没有同时设置频率调控器
- 需要单独配置才能启用 `performance` 模式

### 为什么需要 performance？

| 模式 | 特性 | 延迟 | 功耗 |
|------|------|------|------|
| **performance** | 固定最大频率，无动态调整 | ✓ 低（毫秒级） | ✗ 高 |
| **powersave** | 动态调整频率，低负载降频 | ✗ 高（频率切换延迟） | ✓ 低 |
| **ondemand** | 按需提频 | ⚠️ 中（有延迟尖峰） | ⚠️ 中 |

**RT 系统必须用 performance**，否则 CPU 动态降频会导致处理延迟不可控。

## 文件位置

| 文件 | 说明 |
|------|------|
| `tools/setup_cpu_freq.sh` | 快速配置脚本 |
| `tools/setup_rt_full.sh` | 完整 RT 部署脚本（已集成此步骤） |

## 故障排除

### 脚本返回"Permission denied"

```bash
# 确保使用 sudo
sudo bash tools/setup_cpu_freq.sh
```

### "cpupower: command not found"

脚本会自动使用直接 sysfs 写入，无需手动安装。

若要安装 cpupower：

```bash
sudo apt install linux-tools-generic
# 或
sudo apt install cpupower
```

### 设置后仍然是 powersave

检查是否有其他服务覆盖设置：

```bash
# 查看当前状态
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# 检查 powermanagement 服务
sudo systemctl status power-profiles-daemon
sudo systemctl status tlp

# 若有这些服务，可以禁用：
sudo systemctl disable power-profiles-daemon
sudo systemctl disable tlp
sudo systemctl stop power-profiles-daemon
sudo systemctl stop tlp
```

### BIOS 设置冲突

有些嵌入式平台的 BIOS 可能强制使用省电模式。检查 BIOS 设置中是否有：
- CPU Power Management
- Turbo Boost / Speed Step
- C-State / P-State

若在 BIOS 中禁用了 CPU 频率管理，脚本无法修改。

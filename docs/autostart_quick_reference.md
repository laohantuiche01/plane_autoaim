# 自启动配置快速参考

## 🚀 一句话快速开始

```bash
bash tools/setup_autostart_service.sh
```

选择 **[1] systemd** 和你的机型，完成！下次开机自动启动。

---

## 📋 常用命令

| 操作 | 命令 |
|------|------|
| **交互式配置** | `bash tools/setup_autostart_service.sh` |
| **systemd 方式配置** | `bash tools/setup_autostart_service.sh --method systemd --robot-type default` |
| **Desktop 方式配置** | `bash tools/setup_autostart_service.sh --method desktop` |
| **查看启动状态** | `systemctl --user status ckyf-vision` |
| **启动服务** | `systemctl --user start ckyf-vision` |
| **停止服务** | `systemctl --user stop ckyf-vision` |
| **实时日志** | `journalctl --user -u ckyf-vision -f` |
| **手动前台启动** | `bash tools/quick_start_vision.sh infantry_4` |
| **卸载自启动** | `bash tools/setup_autostart_service.sh --uninstall` |

---

## 🎯 快速选择

### 场景 A：实车自动启动（推荐）

```bash
bash tools/setup_autostart_service.sh --method systemd --robot-type infantry_4
sudo reboot  # 重启验证
```

### 场景 B：开发快速启动

```bash
bash tools/quick_start_vision.sh --robot-type default
```

### 场景 C：后台启动 + 实时监控

```bash
systemctl --user start ckyf-vision
journalctl --user -u ckyf-vision -f
```

---

## 📍 文件位置

| 内容 | 位置 |
|------|------|
| systemd 服务 | `~/.config/systemd/user/ckyf-vision.service` |
| Desktop 配置 | `~/.config/autostart/ckyf-vision.desktop` |
| 启动脚本 | `~/.local/bin/ckyf-vision-start.sh` |
| 日志文件 | `~/.ckyf_vision_autostart/vision.log` |

---

## ❓ 故障排除

**Q: 启动失败？**
A: `journalctl --user -u ckyf-vision -n 50 --no-pager`

**Q: 修改了机型参数？**
A: `nano ~/.config/systemd/user/ckyf-vision.service` 修改 `ROBOT_TYPE=xxx`，然后 `systemctl --user daemon-reload && systemctl --user restart ckyf-vision`

**Q: 如何禁用自启动？**
A: `systemctl --user disable ckyf-vision` 或 `bash tools/setup_autostart_service.sh --uninstall`

---

详细文档见 `docs/autostart_service_guide.md`

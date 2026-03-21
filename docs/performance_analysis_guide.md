# 性能分析与监控工具指南

在开发高频视觉流水线（如 250Hz 图像处理）时，系统级的性能监控对于排查瓶颈、验证零拷贝和绑核优化至关重要。本文档详细介绍了常用的性能分析软件的安装与使用方法。

## 1. 核心监控工具概览

- **htop**: 交互式进程和系统资源监控工具，提供直观的全局视角。
- **pidstat**: 进程/线程级的资源统计分析工具，提供精确的量化指标（如上下文切换、CPU 使用率）。
- **perf**: Linux 内核级的性能分析工具，用于深入分析缓存未命中、指令执行效率等硬件级指标。
- **ROS 2 CLI**: ROS 2 自带的命令行工具，用于监控话题的频率、延迟和带宽。

---

## 2. 工具安装

在 Ubuntu 等基于 Debian 的系统上，可以通过以下命令安装这些工具：

```bash
sudo apt update
sudo apt install -y htop sysstat linux-tools-common linux-tools-generic linux-tools-$(uname -r)
```
*(注：`sysstat` 包含了 `pidstat` 工具。)*

---

## 3. 详细使用指南

### 3.1 htop - 实时全局与多核监控

`htop` 是 `top` 命令的增强版，非常适合用来验证我们的 CPU 绑核（CPU Affinity）策略是否生效。

**基本用法：**
```bash
htop
```

**关键操作与观测点：**
1. **观察绑核效果**：在顶部的 CPU 负载条中，如果您在 `vision.launch.py` 中配置了 `taskset -c 0,1,2,3`，您应该能明显看到这 4 个核心的负载波动，而其他核心相对空闲。
2. **查看线程拓扑**：
   - 按下 `F2` 进入设置 (Setup)。
   - 选择 `Display options`。
   - 勾选 `Tree view` (树状视图) 和 `Show custom thread names` (显示自定义线程名)。
   - 按 `F10` 退出。
   - 这样您就能在进程列表中清晰地看到 `component_container_isolated` 进程下分裂出的各个节点线程（如相机读取、检测器、解算器）。
3. **快捷键**：
   - `F3`: 搜索进程名称（如输入 `vision`）。
   - `F4`: 过滤进程。
   - `F9`: 发送信号（如结束进程）。

### 3.2 pidstat - 精确的线程级度量

当您需要精确量化某个进程或其内部线程的性能时，`pidstat` 是最佳选择。

**查找目标进程的 PID：**
```bash
pgrep -f component_container_isolated
```
假设 PID 为 `12345`。

**1. 监控线程级 CPU 使用率：**
```bash
# -t: 显示线程级别的统计
# -p: 指定进程 ID
# 1: 每 1 秒刷新一次
pidstat -t -p 12345 1
```
*观测点*：您可以分别看到相机驱动线程、检测器线程和解算器线程的 `%usr` 和 `%system` 占用，判断哪个环节是计算瓶颈。

**2. 监控上下文切换（关键指标）：**
```bash
# -w: 报告任务切换情况
pidstat -wt -p 12345 1
```
*观测点*：
- **cswch/s (自愿上下文切换)**：线程主动让出 CPU（例如等待下一帧图像到来）。
- **nvcswch/s (非自愿上下文切换)**：线程被迫让出 CPU（例如时间片耗尽或被更高优先级任务抢占）。**如果在绑核状态下此数值较高，说明系统存在严重的资源竞争，您的视觉线程正在被其他进程干扰。**

**3. 监控内存与缺页中断：**
```bash
# -r: 报告内存使用情况
pidstat -tr -p 12345 1
```
*观测点*：`minflt/s` 和 `majflt/s` (次要/主要缺页中断)。在使用零拷贝（Intra-process/Iceoryx）且内存分配稳定后，这些数值应该极低。如果数值居高不下，可能意味着系统在频繁分配和销毁大块内存（如深拷贝图像）。

### 3.3 perf - 硬件级缓存与指令分析

如果您需要验证绑核是否真正提高了缓存命中率，可以使用 `perf`。

**统计缓存未命中情况（需要 sudo 权限）：**
```bash
# 监测整个容器进程 10 秒内的硬件指标
sudo perf stat -e cache-references,cache-misses,cycles,instructions -p 12345 sleep 10
```
*观测点*：
- **cache-misses**: 缓存未命中占 cache-references 的百分比。绑核（`taskset`）的主要目的就是降低这个百分比。如果未命中率显著下降，说明 OpenCV 等高强度内存访问算法正在高效利用 L2/L3 缓存。
- **instructions per cycle (IPC)**: 每周期执行指令数。通常 IPC 越高，CPU 执行效率越高。

### 3.4 ROS 2 CLI - 端到端延迟与频率分析

对于 ROS 2 原生通信，命令行工具提供了最直接的话题（Topic）状态监控。

**操作前准备：**
确保环境变量已加载（特别是在启用了 CycloneDDS + 共享内存发现的情况下）：
```bash
source /home/mijiao/ckyf_vision/.envrc
```

**1. 监控话题频率：**
```bash
# 查看检测器输出的话题频率是否达到预期的 250Hz
ros2 topic hz /armor_detector/armors
```

**2. 监控端到端延迟：**
*(前提：消息结构必须包含 `std_msgs/Header` 且时间戳打制正确)*
```bash
# 查看从图像捕获到解算完成的总延迟
ros2 topic delay /armor_solver/aim
```

**3. 监控话题带宽：**
```bash
# 查看调试图像等大话题的网络/内存吞吐量
ros2 topic bw /armor_solver/debug_image
```

---

## 4. 性能调优总结与建议

1. **宏观定位**：先用 `htop` 看整体 CPU 负载是否均衡，有没有跑满某个核心导致系统卡顿。
2. **微观诊断**：用 `pidstat -wt` 检查非自愿上下文切换 (`nvcswch/s`)。如果视觉核心的切换过高，考虑调整系统优先级或优化 `taskset` 的绑核策略，避开中断密集的 CPU0。
3. **算法耗时**：依赖代码中嵌入的 `RCLCPP_INFO` 探针（如 `Avg Process Time: X.XX ms`）。在 250Hz 需求下，单帧处理时间必须严格控制在 **4ms** 以内。
4. **内存验证**：结合日志中的指针地址打印和 `pidstat -r` 的缺页中断率，确认零拷贝机制稳定运行。
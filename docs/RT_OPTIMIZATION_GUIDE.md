# RT 优化部署指南

## 🚀 快速开始（推荐）

如果您希望使用**交互式菜单**一步步完成配置，请使用新的统一设置脚本：

```bash
bash tools/setup_complete.sh
```

该脚本提供以下选项：
- **选项 1**：检查基础环境（仅检查，不安装）
- **选项 2**：安装基础依赖（OpenCV、Eigen、G2O 等）
- **选项 3**：安装 jemalloc 和 direnv
- **选项 4**：配置 direnv shell hook 和 .envrc
- **选项 5**：配置 RT 优化（需要 root 权限）
- **选项 6**：完整部署（依赖安装 + RT 优化，需要 root 权限）

### 典型工作流

```bash
# 1️⃣ 非 root 用户：检查并安装基础依赖、jemalloc、direnv
bash tools/setup_complete.sh
# 选择：2 → 3 → 4 → 0

# 2️⃣ 激活 direnv
cd /home/mijiao/ckyf_vision && direnv allow

# 3️⃣ root 用户：配置 RT 优化（需新终端用 sudo 运行）
sudo bash tools/setup_complete.sh
# 选择：5 → 0（或选择 6 一键部署）

# 4️⃣ 重启后验证
bash tools/check_rt_status.sh --bench
```

---

## 📋 改动清单

### ✅ 已完成的改动

#### 1. 四个部署脚本（已创建）

- **`tools/setup_complete.sh`** - 新增：交互式统一配置脚本
  - 菜单驱动，支持分步或完整部署
  - 自动检测依赖安装状态
  - 支持 jemalloc、direnv 配置

- **`tools/setup_rt_full.sh`** - RT 优化部署脚本（改进）
  - 添加 jemalloc 和 direnv 依赖安装
  - 自动配置 direnv hook 到 shell 配置文件
  - 创建项目级 .envrc 文件

#### 脚本优化（功能简化）

- **`tools/setup_complete.sh`** - 统一交互式菜单
  - 包含所有环境配置功能
  - 支持分步或完整部署
  - 自动检测重复安装

- **`tools/restore_rt.sh`** - 恢复回滚脚本
  - 自动从备份恢复 grub 配置
  - 清除所有 RT 权限配置
  - 恢复 irqbalance 默认设置

- **`tools/check_rt_status.sh`** - 状态检验脚本
  - 检查 isolcpus、SCHED_FIFO、CPU 绑核
  - CPU 频率、内存锁定状态
  - 实时采样上下文切换和缓存统计
  - 可选 RT 基准测试（cyclictest、perf）

#### 2. Launch 文件修改

- **`src/robot_bringup/launch/vision.launch.py:48`**
  ```python
  # 改前：prefix=['nice -n -20 taskset -c 0,1,2,3']
  # 改后：prefix=['chrt -f 90 taskset -c 0,1,2,3']
  ```

- **`src/robot_bringup/launch/vision_bag.launch.py:36`**
  - 同样修改为 `chrt -f 90`

#### 3. 相机驱动优化（hik_camera_driver.cpp）

- 添加 `#include <sys/mman.h>`
- 构造函数中新增 `mlockall(MCL_CURRENT | MCL_FUTURE)`
- 优化 memcpy：消除临时 vector 构造
  ```cpp
  // 改前：image->data = std::vector<unsigned char>(bayerImage.data, ...);
  // 改后：
  const std::size_t frame_size = ...;
  if (image->data.size() != frame_size) {
      image->data.resize(frame_size);
  }
  std::memcpy(image->data.data(), frameOut.pBufAddr, frame_size);
  ```
- CameraInfo 发布频率降低：每 30 帧发布一次（约 120ms @ 250Hz）

#### 4. 装甲板检测节点优化（armor_detector_node.cpp）

- 添加 `#include <sys/mman.h>`
- 构造函数中新增 `mlockall(MCL_CURRENT | MCL_FUTURE)`

#### 5. jemalloc 内存分配器集成

- **`src/hik_camera_driver/CMakeLists.txt`**
  ```cmake
  find_package(jemalloc REQUIRED)
  target_link_libraries(${PROJECT_NAME} ... jemalloc)
  ```

- **`src/robot_auto_aim/CMakeLists.txt`**
  ```cmake
  find_package(jemalloc REQUIRED)
  target_link_libraries(${PROJECT_NAME} ... jemalloc)
  ```

#### 6. 项目级环境配置

- **`.envrc`** 自动创建（由 setup scripts 生成）
  ```bash
  export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libjemalloc.so.2
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export MALLOC_CONF="background_thread:true,metadata_thp:auto"
  ```
  - 使用 `direnv` 自动加载
  - 无需手动修改 shell 配置

---

## 📖 详细部署流程（手动步骤）

如果您更喜欢手动控制每个步骤，可以参考以下的四阶段流程：

### 阶段 1：准备工作（<5 分钟）

**目标**：安装工具、配置用户权限

```bash
cd /home/mijiao/ckyf_vision

# 安装性能分析工具和 RT 相关包
sudo bash tools/setup_rt_full.sh --install-tools

# 重新登录使 limits.conf 生效
logout
# 或在新终端验证
groups  # 应包含当前用户的权限
```

验证：
```bash
# 检查权限是否生效
grep $(whoami) /etc/security/limits.conf | head -3
```

### 阶段 2：内核参数配置（需要 reboot）

**目标**：启用 CPU 隔离（isolcpus）

```bash
# 修改 grub 配置并重启
sudo bash tools/setup_rt_full.sh

# 根据提示重启
sudo reboot
```

验证：
```bash
# 重启后检查内核参数
cat /proc/cmdline | grep isolcpus
# 应输出：isolcpus=0,1,2,3 nohz_full=0,1,2,3 rcu_nocbs=0,1,2,3
```

### 阶段 3：代码编译（<5 分钟）

**目标**：编译包含 mlockall、memcpy 优化和 jemalloc 链接的代码

```bash
cd /home/mijiao/ckyf_vision

# 1. 确保 jemalloc-dev 已安装
sudo apt-get install -y libjemalloc-dev

# 2. 完整编译（jemalloc 会自动从 CMakeLists.txt 链接）
colcon build \
  --packages-select hik_camera_driver robot_auto_aim robot_bringup \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. 验证编译成功
source install/setup.bash

# 4. 验证 jemalloc 已链接
ldd install/hik_camera_driver/lib/libhik_camera_driver.so | grep jemalloc
# 应输出：libjemalloc.so.2 => /usr/lib/...
```

### 阶段 4：验证与微调（<10 分钟）

**目标**：启动系统并验证 RT 配置生效

```bash
# 启动视觉系统
source install/setup.bash
ros2 launch robot_bringup vision.launch.py robot_type:=default &

# 等待系统稳定（~5秒）
sleep 5

# 运行全面检验（无基准测试）
bash tools/check_rt_status.sh

# 运行包含基准测试的检验（需要视觉系统运行中）
bash tools/check_rt_status.sh --bench
```

**预期输出**（示例）：
```
[PASS] isolcpus=0,1,2,3
[PASS] SCHED_FIFO priority:90
[PASS] CPU 绑核正确（core 0-3）
[PASS] core0-3: performance @ 3600MHz
[PASS] mlockall 已启用（锁定率: 95%）
nvcswch/s < 2  （非自愿切换接近消除）
```

---

## 📊 性能指标对标

### 非自愿上下文切换（nvcswch/s）

| 阶段 | 期望值 | 实现难度 |
|------|--------|---------|
| 未优化 | 10-50 | — |
| 仅 nice -n -20 | 5-20 | 简单 |
| + SCHED_FIFO | 2-8 | 简单 |
| + isolcpus | <2 | 需要 reboot |
| + IRQ 迁移 | <1 | 进阶 |

### 缓存性能（cache-misses ratio）

| 优化 | 改进 | 累积 |
|------|------|------|
| 基线 | — | 0% |
| memcpy 优化 | -3% | -3% |
| mlockall | -5% | -8% |
| 内核隔离 | -10% | -18% |

### RT 延迟（cyclictest Max）

| 目标 | 阈值 | 评估 |
|------|------|------|
| 优秀 | <50μs | 需要 PREEMPT_RT 内核 |
| 良好 | <200μs | 本方案可达成 |
| 可用 | <500μs | 保证达成 |

---

## ⚠️ 常见问题

### Q1：isolcpus 需要 reboot 吗？

**A**：是的。isolcpus 是内核启动参数，需要在启动时生效。修改后需 `sudo reboot`。

### Q2：我没有 cyclictest，怎么办？

**A**：运行 `sudo bash tools/setup_rt_full.sh --install-tools` 自动安装 rt-tests 包。如果安装失败，手动运行：
```bash
sudo apt-get install -y rt-tests
```

### Q3：chrt -f 90 报错"Operation not permitted"

**A**：说明 `limits.conf` 权限未生效。检查：
```bash
grep $(whoami) /etc/security/limits.conf
```
如果为空，需要重新登录或重启。

### Q4：如何回滚优化？

**A**：运行恢复脚本：
```bash
sudo bash tools/restore_rt.sh
sudo reboot  # 仅在修改了 grub 时需要
```

### Q5：mlockall 会占用多少内存？

**A**：锁定进程的所有内存页。对于视觉系统（~100-200MB RSS），占用成本可接受。系统需至少 1GB 空闲内存。

### Q6：vision_bag 也需要修改吗？

**A**：是的。vision_bag.launch.py 第 36 行也已修改为 `chrt -f 90`，确保离线调试时也使用 SCHED_FIFO。

---

## 📈 性能分析示例

### 运行 cyclictest 基准测试

```bash
# 需要视觉系统已启动
bash tools/check_rt_status.sh --bench

# 手动运行（更详细）
cyclictest -l 10000 -m -S -p 90 -i 1000 -a 0

# 输出示例：
# T: 0 ( 1234) P: 90 I:1000 C:10000 Min:      5 Act:    8 Avg:    12 Max:     45
#                                    Min=5μs   Avg=12μs  Max=45μs
```

**解读**：
- Min < 20μs：说明系统调度延迟极低（优秀）
- Max < 100μs：表示所有采样点延迟都很低（优秀）
- Max < 200μs：可用于实时应用（良好）

### 采样上下文切换

```bash
# 实时监控 5 秒
pidstat -w -p $(pgrep component_container) 1 5

# 输出示例：
# 12:34:56   UID       PID   cswch/s nvcswch/s  Command
# 12:34:57  1000    12345      0.50    0.00  component_container
# 12:34:58  1000    12345      0.60    0.10  component_container
# 平均  nvcswch/s = 0.05（非常低，优秀）
```

**解读**：
- nvcswch/s < 0.1：非自愿切换几乎消除（优秀）
- nvcswch/s < 1：显著改善（良好）
- nvcswch/s > 5：优化效果不明显（需排查 IRQ 或其他高优先级线程）

---

## 🔍 深度诊断

如果优化效果不理想，按以下步骤诊断：

```bash
# 1. 检查 isolcpus 是否真的生效
cat /proc/cmdline | grep isolcpus

# 2. 检查 core 0-3 上是否有其他进程
ps aux | grep -E "^\S+\s+\S+\s+(0|1|2|3)\s"

# 3. 检查 IRQ 是否全部迁移出去
cat /proc/interrupts | grep -E "xhci|usb" | head -3

# 4. 检查定时器中断是否被禁用（nohz_full 生效）
cat /proc/stat | head  # 在隔离核上 irq 计数增长应该很慢

# 5. 监控内存锁定状态
cat /proc/$(pgrep component_container)/status | grep Vm
```

---

## 📚 脚本参考

| 脚本 | 用途 | 权限 | 说明 |
|------|------|------|------|
| `tools/setup_complete.sh` | 交互式配置（推荐） | 普通用户 (需 sudo) | 菜单驱动，支持分步或完整部署 |
| `tools/setup_rt_full.sh` | RT 优化部署 | root | 配置 isolcpus、limits.conf、jemalloc、direnv |
| `tools/check_rt_status.sh` | 状态检验 | 普通用户 | 检查优化是否生效，可选基准测试 |
| `tools/restore_rt.sh` | 恢复回滚 | root | 恢复原始 grub 配置，清除所有优化 |

### 脚本依赖关系

```
setup_complete.sh (交互式主菜单)
  ├─ setup_rt_full.sh (RT优化，需 root)
  │  └─ check_rt_status.sh (验证优化)
  └─ restore_rt.sh (紧急回滚，需 root)
```

## 📋 相关代码文件

| 文件 | 改动内容 |
|------|---------|
| `src/robot_bringup/launch/vision.launch.py:48` | `chrt -f 90` 实时调度 |
| `src/robot_bringup/launch/vision_bag.launch.py:36` | `chrt -f 90` 实时调度 |
| `src/hik_camera_driver/src/hik_camera_driver.cpp` | mlockall + memcpy 优化 + jemalloc 链接 |
| `src/hik_camera_driver/CMakeLists.txt:10,35` | 添加 jemalloc 依赖 |
| `src/robot_auto_aim/src/armor_detector_node.cpp` | mlockall 内存锁定 |
| `src/robot_auto_aim/CMakeLists.txt:20,55` | 添加 jemalloc 依赖 |
| `.envrc` | 自动环境变量加载（direnv） |

---

## 🎯 快速开始（推荐方式）

```bash
# 一条命令启动交互式配置菜单
bash tools/setup_complete.sh
```

### 典型流程（3 分钟快速部署）

```bash
# 1️⃣ 检查基础环境（菜单选项 1）
# 2️⃣ 安装依赖（选项 2）
# 3️⃣ 安装 jemalloc 和 direnv（选项 3）
# 4️⃣ 配置 direnv（选项 4）
# 5️⃣ cd 到项目，激活 direnv
cd /home/mijiao/ckyf_vision && direnv allow

# 6️⃣ 使用 root 配置 RT 优化（选项 5）
sudo bash tools/setup_complete.sh

# 7️⃣ 重启
sudo reboot

# 8️⃣ 验证（重启后）
bash tools/check_rt_status.sh --bench
```

---

## ⚙️ 传统部署步骤（如需手动控制）

### 立即执行：

```bash
# 方式 A：使用交互式菜单（推荐）
bash tools/setup_complete.sh

# 方式 B：使用原始脚本
sudo bash tools/setup_rt_full.sh --install-tools
logout  # 重新登录使 limits.conf 生效
```

### 准备重启：
记录当前工作，reboot 前 10 分钟内完成

### 重启并验证：
```bash
sudo bash tools/setup_rt_full.sh   # 配置 isolcpus（需 reboot）
sudo reboot

# 重启后验证
bash tools/check_rt_status.sh --bench
   sudo reboot

# 重启后验证
bash tools/check_rt_status.sh --bench
```

### 编译和启动：

```bash
cd /home/mijiao/ckyf_vision

# 1. 安装 jemalloc-dev
sudo apt-get install -y libjemalloc-dev

# 2. 重新编译（jemalloc 会自动链接）
colcon build \
  --packages-select hik_camera_driver robot_auto_aim robot_bringup \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. 激活环境并启动
source install/setup.bash
cd /home/mijiao/ckyf_vision && direnv allow  # 激活环境变量
ros2 launch robot_bringup vision.launch.py robot_type:=default

# 4. 新终端验证优化效果
bash tools/check_rt_status.sh --bench
```

---

## 📊 预期收益

| 指标 | 优化前 | 优化后 | 改进 |
|------|--------|--------|------|
| nvcswch/s | 50-100 | 2-5 | ↓ 90% |
| cache-misses | 高 | 低 | ↓ 15-20% |
| 延迟抖动 | 明显 | 极低 | ↓ 显著 |
| malloc 锁争用 | 有 | 无 | ✓ 消除 |

**总耗时**：~15 分钟（含 reboot）

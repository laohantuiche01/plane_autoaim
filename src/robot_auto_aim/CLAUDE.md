[根目录](../../CLAUDE.md) > [src](../) > **robot_auto_aim**

# robot_auto_aim

## 模块职责

自瞄系统的**核心算法包**，实现从图像到三维状态预测的完整流水线。包含两个生命周期节点：

- **ArmorDetectorNode**：装甲板检测与三维位姿估计
- **ArmorSolverNode**：UKF 状态跟踪、轨迹生成与火控决策

两个节点均实现 `rclcpp_lifecycle::LifecycleNode` 接口，通过 `robot_manager` 进行生命周期管理。

---

## 节点列表

### ArmorDetectorNode（装甲板检测节点）

| 属性 | 内容 |
|------|------|
| 节点名称 | `armor_detector` |
| Plugin 名称 | `robot_auto_aim::ArmorDetectorNode` |
| 可执行文件 | `armor_detector_node`（独立）/ `robot_auto_aim_detector_component_node`（Component） |

**订阅话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `image_raw` | `sensor_msgs/msg/Image` | 原始图像（Zero-Copy 接收），仅 Active 状态下订阅 |
| `/image_raw_info` | `sensor_msgs/msg/CameraInfo` | 相机内参，仅接收一次后自动取消订阅 |

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `armor_detector/armors` | `robot_interfaces/msg/Armors` | 检测到的装甲板列表（含 3D 位姿，相机系） |
| `armor_detector/markers` | `visualization_msgs/msg/MarkerArray` | RViz2 可视化标记（debug 模式） |
| `armor_detector/debug_lights` | `robot_interfaces/msg/DebugLights` | 灯条调试数据 |
| `armor_detector/debug_armors` | `robot_interfaces/msg/DebugArmors` | 装甲板调试数据 |
| `armor_detector/binary_img` | `sensor_msgs/msg/Image` | 二值化图像（debug 模式） |
| `armor_detector/number_img` | `sensor_msgs/msg/Image` | 数字分类 ROI 图（debug 模式） |
| `armor_detector/result_img` | `sensor_msgs/msg/Image` | 叠加检测结果的彩色图（debug 模式） |

**关键参数**

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `enemy_color` | `"auto"` | 敌方颜色（`auto` / `red` / `blue`）；`auto` 模式订阅 `/robot/mode` 动态切换（mode=0→红，mode=1→蓝） |
| `binary_thres` | 160 | 二值化阈值（0-255） |
| `use_ba` | true | 是否启用 Bundle Adjustment 精化位姿 |
| `use_pca` | true | 是否启用 PCA 角点校正 |
| `target_frame` | `"odom"` | 坐标系参考帧 |
| `classifier_threshold` | 0.8 | 数字分类置信度阈值 |
| `ignore_classes` | `["negative"]` | 忽略的分类标签 |
| `debug` | true | 是否发布调试话题 |
| `debug_img_freq` | 60.0 | 调试图像发布频率 (Hz) |
| `light.*` | 见 params.yaml | 灯条检测参数（min/max_ratio、max_angle、color_diff_thresh） |
| `armor.*` | 见 params.yaml | 装甲板检测参数（灯条比例、中心距离范围、max_angle） |

**处理流程**

1. 图像 BGR 二值化 -> 灯条轮廓提取（面积、长宽比、角度过滤）
2. 灯条配对 -> 候选装甲板筛选（中心距、倾斜角）
3. 数字 ROI 裁剪 -> LeNet ONNX 模型推理（分类 + 颜色过滤）
4. PnP/BA 解算 3D 位姿 -> 坐标变换至 `odom` 系
5. 发布 `Armors` 消息

---

### ArmorSolverNode（装甲板跟踪与轨迹生成节点）

| 属性 | 内容 |
|------|------|
| 节点名称 | `armor_solver` |
| Plugin 名称 | `robot_auto_aim::ArmorSolverNode` |
| 可执行文件 | `armor_solver_node`（独立）/ `robot_auto_aim_solver_component_node`（Component） |

**订阅话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `armor_detector/armors` | `robot_interfaces/msg/Armors` | 装甲板检测结果，仅 Active 状态下订阅 |
| `image_raw` | `sensor_msgs/msg/Image` | 仅在 debug 模式下订阅，用于投影可视化 |
| `/image_raw_info` | `sensor_msgs/msg/CameraInfo` | 仅 debug 模式下使用一次 |

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `armor_solver/target_state` | `robot_interfaces/msg/TargetState` | UKF 滤波后的完整目标状态（11维/7维） |
| `armor_solver/trajectory` | `robot_interfaces/msg/TargetTrajectory` | 双轨制预测轨迹（定时器驱动，v1.1.0+ 含装甲板宽度） |
| `armor_solver/markers` | `visualization_msgs/msg/MarkerArray` | 目标模型可视化 |
| `armor_solver/measurement` | `robot_interfaces/msg/Measurement` | 原始观测值（debug 模式） |
| `armor_solver/debug_image` | `sensor_msgs/msg/Image` | 投影调试图（debug 模式） |

**关键参数**

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `odom_frame` | `"odom"` | 世界惯性系帧名称 |
| `max_lost_duration` | 1.0 | 目标丢失超时时间 (s) |
| `min_detect_count` | 5 | 初始化所需最小连续检测次数 |
| `bullet_speed` | 25.0 | 子弹初速 (m/s)，用于飞行时间估算 |
| `timer_frequency` | 100.0 | UKF 定时器频率 (Hz)，v1.1.0+ 可配置 |
| `ukf_alpha/beta/kappa` | 0.001/2.0/0.0 | UKF 超参数 |
| `robot.*` | 见下方参数表 | 机器人目标 UKF 参数（Q/R/P0/门限） |
| `outpost.*` | 见下方参数表 | 前哨站目标 UKF 参数 |
| `trajectory.num_points` | 11 | 轨迹序列点数（奇数） |
| `trajectory.dt` | 0.05 | 轨迹点时间间隔 (s) |
| `trajectory.omega_low/high` | 1.5/4.0 | 径向收缩转速阈值 (rad/s) |
| `trajectory.switch_concentration` | 20.0 | 切向软切换余弦指数 n（低速时不使用） |
| `trajectory.hit/aim_delay_offset` | 0.0 | 击打/瞄准点时间偏置补偿 (s) |
| `robot.omega_freeze_thresh` | 0.5 | 低速边界：\|omega\| < 此值时跳过切向融合，aim 使用 best armor |

**robot/outpost 子参数（Q/R/P0/门限）**

| 参数名 | 默认值 (robot / outpost) | 说明 |
|--------|--------------------------|------|
| `sigma_pos` | 20.0 / 0.1 | CWNA 加速度标准差 (m/s²)；outpost 为位置漂移标准差 |
| `sigma_yaw` | 2.0 / 0.3 | 角加速度标准差 (rad/s²)，CWNA |
| `q_geo` | 0.001 / 0.0001 | 几何参数 (r, l, h) 随机游走噪声 |
| `r_range` | 0.01 | 球坐标距离噪声基础方差 (m²) |
| `r_range_k` | 0.5 | 距离噪声随 range² 增长系数 |
| `r_angle` | 0.0003 | 方位角/俯仰角噪声方差 (rad²，常数) |
| `r_yaw` | 0.05 / 0.05 | 基础 yaw 噪声方差 (rad²) |
| `r_yaw_adaptive_factor` | 50.0 | 切换装甲板时 yaw 噪声放大倍数 |
| `r_yaw_viewing_k` | 10.0 | 正对时 yaw 噪声放大系数：R_yaw *= (1 + k·cos²(va)) |
| `mahalanobis_thresh` | 15.0 | 卡方门限（4DoF: 9.49=5%, 7.78=10%） |
| `p0_pos` | 0.1 | 位置初始协方差 |
| `p0_vel` | 10.0 | 速度初始协方差（仅 robot） |
| `p0_yaw` | 0.5 | yaw 初始协方差 |
| `p0_omega` | 100.0 / 10.0 | 角速度初始协方差 |
| `p0_geo` | 0.1 | 几何参数初始协方差 |
| `adaptive_tracking` | false | 是否启用 Sage-Husa 自适应 Q |
| `q_alpha` | 0.1 | 自适应 Q 学习率 |
| `min_update_count` | 5 | 收敛判定最小更新次数 |
| `max_pos_cov` | 2.2 / 3.0 | 位置协方差收敛阈值 |
| `max_yaw_cov` | 1.0 | yaw 协方差收敛阈值 |

**跟踪器状态机**

```
LOST -> DETECTING -> TRACKING -> TEMP_LOST -> TRACKING
                                           -> LOST (超时)
```

**UKF 状态向量**

- 机器人模型（11 维）：`[cx, vx, cy, vy, cz, vz, yaw, omega, r, l, h]`
- 前哨站模型（7 维）：`[cx, cy, cz, yaw, omega, r, h]`（前哨站使用 3 个并行 UKF 解决初态模糊）

---

## 关键依赖

- `rclcpp`、`rclcpp_lifecycle`、`rclcpp_components`
- `robot_interfaces`（Armors、Aim、TargetState、TargetTrajectory）
- `robot_utils`（UKF、PnPSolver、数学工具）
- `robot_ballistics`（BallisticsCalculator，用于飞行时间估算）
- `OpenCV`、`Eigen3`、`tf2_ros`
- `G2O`（Bundle Adjustment 位姿优化）
- `Sophus`（李群/李代数操作，BA 优化内部使用）
- `TBB`（并行加速）
- `fmt`（格式化日志）

---

## 模型文件

| 文件 | 说明 |
|------|------|
| `model/lenet.onnx` | 装甲板数字分类模型（LeNet 架构，ONNX 格式） |
| `model/label.txt` | 分类标签列表（包含 negative、0-9 及字母标签） |

路径通过 `package://robot_auto_aim/model/lenet.onnx` 形式由 `robot_utils::URLResolver` 解析。

---

## 构建与测试

```bash
colcon build --packages-select robot_auto_aim --cmake-args -DCMAKE_BUILD_TYPE=Release
# 仅 ament_lint 检查（copyright/cpplint/uncrustify 已跳过）
colcon test --packages-select robot_auto_aim
```

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.5.0 | 2026-03-25 | 串口 mode 颜色自动切换：enemy_color 新增 "auto" 模式，订阅 /robot/mode 动态更新 detect_color（mode=0→红，mode=1→蓝），默认配置改为 "auto" |
| 1.4.1 | 2026-03-24 | 低速切向融合跳过：当 \|omega\| < omega_freeze_thresh 时，aim_trajectory 使用 best armor（硬切换）而非加权融合，避免瞄准点落在装甲板之间 |
| 1.4.0 | 2026-03-24 | 零角速度可观测性保护：①omega 自适应几何噪声冻结②isDiverged 增强③双假设确认冻结④CONFIRMING_FROZEN 状态+单UKF模式降低计算开销 |
| 1.3.0 | 2026-03-23 | UKF 定时器频率参数化（timer_frequency）；TargetTrajectory 添加 armor_width 字段（用于动态容差） |
| 1.2.0 | 2026-03-22 | 暴露 mahalanobis_thresh 和 P0 初始协方差参数 |
| 1.1.0 | 2026-03-22 | 观测模型重构为球坐标系，CWNA Q 矩阵，修复 isDiverged/CONFIRMING bug |
| 1.0.0 | 2026-03-22 | 初始化文档，自动生成 |

---

## UKF 目标模型详解

### RobotTarget（普通机器人，11维）

**文件：** `src/robot_target.cpp` / `include/robot_auto_aim/robot_target.hpp`

#### 状态向量（STATE_DIM = 11）

| 索引 | 变量 | 含义 |
|------|------|------|
| 0 | `cx` | 圆心 x 坐标（odom 系） |
| 1 | `vx` | 圆心 x 速度 |
| 2 | `cy` | 圆心 y 坐标 |
| 3 | `vy` | 圆心 y 速度 |
| 4 | `cz` | 圆心 z 坐标 |
| 5 | `vz` | 圆心 z 速度 |
| 6 | `yaw` | 参考装甲板朝向角 |
| 7 | `omega` | 旋转角速度 (v_yaw) |
| 8 | `r` | 偶数面（ID 0/2）装甲板半径 |
| 9 | `l` | 奇偶半径差（奇数面半径 = r + l） |
| 10 | `h` | 偶奇面高度差（奇数面 z = cz + h） |

#### 观测向量（MEAS_DIM = 4）：`[range, azimuth, elevation, armor_yaw]`（球坐标）

观测量采用球坐标系，R 矩阵天然对角化，物理意义清晰：
- `range`：目标距离，噪声 `∝ (1 + k·range²)`
- `azimuth/elevation`：方位角/俯仰角，噪声近似常数（由像素精度决定）
- `armor_yaw`：装甲板朝向，噪声随 viewing angle 自适应（正对时放大）

#### 过程模型 `f(x, dt)`（CWNA 匀速直线 + 匀速旋转）

```
cx   += vx  * dt
cy   += vy  * dt
cz   += vz  * dt
yaw  += omega * dt        # 归一化到 [-π, π]
# 其余分量保持不变
```

Q 矩阵使用 CWNA（连续白噪声加速度）模型，每个 `[pos, vel]` 对：
```
Q_axis = σ_pos² × | dt³/3   dt²/2 |
                   | dt²/2   dt    |
```

#### 观测模型 `h(x, id)`（id = 0..3，按 90° 间隔分布）

先计算笛卡尔装甲板位置（`getArmorCartesian`），再转球坐标：
```
# 笛卡尔中间量
angle     = yaw + id * π/2
current_r = (id % 2 == 0) ? r : (r + l)
current_z = (id % 2 == 0) ? cz : (cz + h)
ax = cx - current_r * cos(angle)
ay = cy - current_r * sin(angle)

# 转球坐标输出
range     = sqrt(ax² + ay² + current_z²)
azimuth   = atan2(ay, ax)
elevation = asin(current_z / range)
return (range, azimuth, elevation, angle)
```

#### 双假设初始化机制

初始化时如有几何先验（`init_geo.r > 1e-3`），同时创建 **2 个并行 UKF**：

| 假设 | 含义 | r | l | h |
|------|------|---|---|---|
| UKF[0] | 当前装甲板 = 偶数面（ID 0/2） | `init_geo.r` | `init_geo.l` | `init_geo.h` |
| UKF[1] | 当前装甲板 = 奇数面（ID 1/3） | `r+l` | `-l` | `-h` |

**确认机制**：

CONFIRMING 状态通过三态有限状态机实现可观测性保护：

```
┌──────────────┐  |omega| < thresh  ┌──────────────────┐  |omega| >= thresh  ┌──────────────────┐
│  CONFIRMING  │ ─────────────────> │ CONFIRMING_FROZEN│ ─────────────────>  │  CONFIRMING (恢复)│
│ (双假设并行) │                    │  (单UKF模式)    │                    │    (重新派生)    │
└──────────────┘                    └──────────────────┘                    └──────────────────┘
       │                                                                             │
       │─────────────────────────────────────────────────────────────────────────────┤
       └─ (switch>=2 || update>=100) ──> ┌───────────┐
                                         │ CONFIRMED │
                                         └───────────┘
```

| 状态 | 运行 UKF | 累积误差 | 确认判定 | 目的 |
|------|----------|---------|----------|------|
| `CONFIRMING` | 两个 | 正常累积 | 允许 | 充分可观测，两个假设并行比较 |
| `CONFIRMING_FROZEN` | 仅 best | 不累积 | 禁止 | 低速不可观测，暂停计算，保留较优假设 |
| `CONFIRMED` | 仅 best | — | — | 假设确认，进入单 UKF 跟踪 |

**转换逻辑**：
- **CONFIRMING → CONFIRMING_FROZEN**：`|omega| < omega_freeze_thresh_`（默认 0.5 rad/s）时，选累积误差较小的 UKF，暂停另一个
- **CONFIRMING_FROZEN → CONFIRMING**：`|omega| >= omega_freeze_thresh_` 时，从活跃 UKF 重新派生暂停假设，重置累积误差和 switch count（保证公平比较），恢复双 UKF
- **CONFIRMING → CONFIRMED**：`switch_count >= 2 || update_count >= 100`（仅在 CONFIRMING 状态，FROZEN 中不允许）

**重新派生算法**（FROZEN→CONFIRMING）：两个假设共享旋转中心 `(cx, cy, cz)` 和运动学 `(vx, vy, vz, yaw, omega)`，仅几何参数差异（90°旋转）：
```cpp
x_alt(8) = x_active(8) + x_active(9);  // r_alt = r + l
x_alt(9) = -x_active(9);                // l_alt = -l
x_alt(10) = -x_active(10);              // h_alt = -h
```
重置几何协方差 `P_alt(8..10, 8..10) = p0_geo_`，清除几何-运动学交叉项。

**性能优化**：FROZEN 时仅运行一个 UKF，计算开销减少 ~50%。

#### 噪声矩阵

- **Q（过程噪声）**：
  - 位置-速度对：CWNA 模型 `Q_pos_vel = σ² × [dt³/3, dt²/2; dt²/2, dt]`
  - 角度-角速度对：CWNA 模型 `Q_yaw_omega = σ² × [dt³/3, dt²/2; dt²/2, dt]`
  - **几何参数** `r, l, h`：简单随机游走，但 **omega 自适应缩放** `Q_geo = q_geo × min(1, |omega|/omega_freeze_thresh_) × dt`
    - 当 `|omega| >= thresh` 时：完整的 q_geo，正常几何参数更新
    - 当 `|omega| < thresh` 时：q_geo 线性缩小，防止几何与位置混淆导致漂移
    - 物理意义：纯平移时几何参数不变，不应有随机游走；恢复旋转后参数可重新收敛
- **R（观测噪声）**：球坐标对角阵 `diag(r_range*(1+k*d²), r_angle, r_angle, r_yaw*(1+k*cos²(va)))`
  - 距离噪声随距离平方增长（PnP 深度误差特性）
  - 方位角/俯仰角噪声近似常数（像素精度决定）
  - yaw 噪声随 viewing_angle 自适应（正对时放大，解决 PnP 几何退化）
  - 切换装甲板时 yaw 噪声 × `r_yaw_adaptive_factor`
- **Mahalanobis 门限**：`mahalanobis_thresh`（默认 15.0），用于异常观测拒绝
- 支持 `adaptive_tracking` 模式（Sage-Husa 在线 Q 估计）

#### 收敛 / 发散判断

| 类型 | 条件 |
|------|------|
| 收敛 `isConverged()` | `update_count > min_update_count` 且位置协方差 `< max_pos_cov (3.0)` 且 yaw 协方差 `< max_yaw_cov (1.0)` |
| 发散 `isDiverged()` | 位置协方差 `> 1000` 或目标距离 `> 400 m` 或 `r < 0.05 / r > 0.6` 或**几何协方差** `P(r/l/h) > 1.0 m²` |

**发散检测增强**（v1.4.0）：新增几何协方差 `P(r)、P(l)、P(h) > 1.0 m²` 检测，捕获低可观测性下的缓慢漂移。

#### 轨迹生成中的低速边界处理（v1.4.1）

双轨制轨迹在生成时分别计算 `true_trajectory`（物理击打点，无切向融合）和 `aim_trajectory`（平滑瞄准点）。在低速纯平移场景（|omega| < omega_freeze_thresh ≈ 0.5 rad/s），切向融合可能在装甲板面过渡角（~45°）产生不在任何实际装甲板上的插值位置，导致 aim 点偏离目标。

**改进**（v1.4.1）：当 `|omega| < omega_freeze_thresh` 时，aim_trajectory 亦采用硬切换（选择 best armor），与 true_trajectory 行为一致。物理含义：无旋转时目标不改变朝向，无需装甲板间平滑过渡，aim 应精确跟踪同一目标装甲板。

**参数**：
- `robot.omega_freeze_thresh`：低速边界阈值（默认 0.5 rad/s），同时用于几何参数冻结和轨迹融合跳过
- `trajectory.switch_concentration`：切向软切换余弦指数 n（仅在 |omega| >= thresh 时启用）

---

### OutpostTarget（前哨站，7维）

**文件：** `src/outpost_target.cpp` / `include/robot_auto_aim/outpost_target.hpp`

#### 状态向量（STATE_DIM = 7）

| 索引 | 变量 | 含义 |
|------|------|------|
| 0 | `cx` | 圆心 x |
| 1 | `cy` | 圆心 y |
| 2 | `cz` | 圆心 z |
| 3 | `yaw` | 参考装甲板朝向角 |
| 4 | `v_yaw` | 旋转角速度 |
| 5 | `r` | 装甲板半径 |
| 6 | `h` | 相邻装甲板高度差 |

#### 过程模型 `f(x, dt)`（纯旋转 CWNA）

```
yaw += v_yaw * dt    # 归一化到 [-π, π]
# 其余分量保持不变（cx/cy/cz/v_yaw/r/h 不变）
```

位置 Q 为随机游走（`sigma_pos² * dt`），yaw-v_yaw 对使用 CWNA 交叉项。

**几何参数 Q 自适应**（v1.4.0）：同 RobotTarget，`r、h` 随机游走过程噪声按 omega 自适应缩放，防止纯平移时参数漂移。

#### 观测模型 `h(x, id)`（id = 0..2，按 120° 间隔，球坐标输出）

```
ANGULAR_OFFSET = 2π/3
angle = yaw + id * ANGULAR_OFFSET
ax = cx - r * cos(angle)
ay = cy - r * sin(angle)
az = cz - id * h

# 转球坐标
range     = sqrt(ax² + ay² + az²)
azimuth   = atan2(ay, ax)
elevation = asin(az / range)
return (range, azimuth, elevation, angle)
```

#### 三假设初始化机制

初始化时创建 **3 个并行 UKF**，对应当前观测装甲板为第 0/1/2 个的三种假设：

```
UKF[k]: yaw_init = armor.yaw - k * (2π/3)
        cz_init  = armor.position.z + k * h_init
```

**确认机制**：观测到 3 个不同装甲板 ID（`observed_ids_.size() >= 3`）后，选累积误差最小的 UKF 确认

#### 收敛 / 发散判断

| 类型 | 条件 |
|------|------|
| 收敛 `isConverged()` | `update_count > min_update_count` 且位置协方差 `< 2.0` 且 yaw 协方差 `< 1.0` |
| 发散 `isDiverged()` | 位置协方差 `> 100` 或目标距离 `> 40 m` 或 `r` 超出 [0.1, 0.5] 或 `h` 超出 [0.05, 0.2] |

**备注**（v1.4.0）：OutpostTarget 三假设 UKF 不适用低速冻结优化（前哨站通常快速旋转，omega 始终较高）。

---

## 相关文件清单

| 文件 | 说明 |
|------|------|
| `src/armor_detector_node.cpp` | 检测节点实现（生命周期、图像回调） |
| `src/armor_solver_node.cpp` | 求解节点实现（UKF 更新、轨迹生成、定时器） |
| `src/armor_detector.cpp` | 检测算法核心（二值化、灯条提取、装甲板匹配） |
| `src/armor_pose_estimator.cpp` | PnP/BA 位姿估计 |
| `src/ba_solver.cpp` | Bundle Adjustment 实现（G2O） |
| `src/graph_optimizer.cpp` | 图优化辅助 |
| `src/light_corner_corrector.cpp` | PCA 灯条角点校正 |
| `src/number_classifier.cpp` | ONNX 模型推理（数字分类） |
| `src/tracker.cpp` | 目标跟踪器状态机实现 |
| `src/robot_target.cpp` | 机器人目标 UKF 模型（11 维，omega 自适应 q_geo，三态CONFIRMING） |
| `src/outpost_target.cpp` | 前哨站目标 UKF 模型（7 维，3 假设并行，omega 自适应 q_geo） |
| `include/robot_auto_aim/types.hpp` | Armor、Light 等核心数据类型 |
| `include/robot_auto_aim/tracker.hpp` | Tracker 类声明 |
| `include/robot_auto_aim/target_base.hpp` | TargetBase 抽象基类 |
| `README.md` | 模块说明文件 |

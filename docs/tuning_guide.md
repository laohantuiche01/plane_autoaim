# 视觉预测与弹道结算系统参数调试指南

本指南涵盖了 `robot_auto_aim` (ArmorSolverNode) 和 `robot_ballistics` (BallisticsNode) 的核心参数说明、调试建议及常见问题排查。

---

## 1. ArmorSolverNode (目标跟踪与轨迹预测)

该节点负责卡尔曼滤波、状态估计以及未来轨迹序列的生成。

### 全局参数
| 参数名 | 默认值 | 说明 | 调试建议 |
| :--- | :--- | :--- | :--- |
| `odom_frame` | `odom` | 惯性参考系名称 | 确保与 SLAM/定位发布的坐标系一致。 |
| `max_lost_duration` | `1.0` | 目标丢失后保留状态的最长时间 (s) | 增大可提高遮挡容忍度。 |
| `min_detect_count` | `5` | 进入追踪状态所需的最小连续识别次数 | 增加可过滤误识别。 |

### 针对性参数 (`robot.*` 或 `outpost.*`)
| 参数名 | 建议范围 | 说明 | 现象与调优 |
| :--- | :--- | :--- | :--- |
| `q_x / y / z` | `0.001 - 0.1` | 中心位置过程噪声 | **抖动剧烈**：减小此值；**跟随迟钝**：增大此值。 |
| `q_vx / vy / vz` | `0.1 - 1.0` | 中心速度过程噪声 (仅机器人) | 目标急停急转后**丢失严重**：增大此值。 |
| `q_yaw / v_yaw` | `0.01 - 0.5` | 自旋角度/角速度过程噪声 | 影响旋转预测。**预测角度落后**：增大 `q_v_yaw`。 |
| `r_x / y / z` | `0.1 - 1.0` | 观测噪声基数 | 增大可使滤波更平滑，但响应变慢。 |
| `z_scale_coeff` | `5.0 - 20.0` | Z轴（深度）噪声缩放 | 视觉深度数据通常较脏，通过此系数压低 Z 轴权重。 |

### 轨迹生成参数 (`trajectory.*`)
| 参数名 | 默认值 | 说明 | 调优建议 |
| :--- | :--- | :--- | :--- |
| `bullet_speed` | `25.0` | 预期弹速 (m/s) | **核心参数**。必须与实时弹速一致，否则预测点会打前/打后。 |
| `trajectory.hit_delay_offset` | `0.0` | 击打点额外延时补偿 (s) | 修正开火时机的物理偏差。**打在目标后方**：增大此值。 |
| `trajectory.aim_delay_offset` | `0.0` | 瞄准点额外延时补偿 (s) | 修正云台跟随的超前/滞后感。 |
| `num_points` | `11` | 轨迹序列点数 (奇数) | 影响滤波窗口大小。点数越多越平滑，但计算开销增大。 |
| `dt` | `0.05` | 序列点时间间隔 (s) | 决定了滤波器能“看”多远。通常维持 `0.02 - 0.05`。 |
| `omega_low / high` | `1.5 / 4.0` | 小陀螺收缩阈值 (rad/s) | 决定何时开始瞄准中心。**云台跟不上**：降低此阈值。 |
| `switch_concentration` | `20.0` | 装甲板切换集中度 | **越大**：瞄准点锁定越死；**越小**：切换过程越平稳。 |

---

## 2. BallisticsNode (弹道结算与指令发布)

该节点负责抛物线重力补偿、基于时间戳的动态多项式平滑滤波以及开火判定。

### 核心解算参数
| 参数名 | 默认值 | 说明 | 调优建议 |
| :--- | :--- | :--- | :--- |
| `bullet_speed` | `25.0` | 弹速 (m/s) | 用于计算抛物线仰角补偿。 |
| `gimbal_frame` | `gimbal_link` | 云台参考系 | 角度解算的目标系（通常为 pitch 轴中心）。 |
| `hit_yaw_offset` | `0.0` | 击打点角度偏置 (rad) | 修正物理安装偏差，影响开火判定。 |
| `hit_pitch_offset` | `0.0` | 击打点角度偏置 (rad) | 修正物理安装偏差，影响开火判定。 |
| `aim_yaw_offset` | `0.0` | 瞄准点角度偏置 (rad) | 最终发给云台的指令偏置。 |
| `aim_pitch_offset` | `0.0` | 瞄准点角度偏置 (rad) | 最终发给云台的指令偏置。 |

### 开火判定 (Firing Decision)
| 参数名 | 默认值 | 说明 | 调优建议 |
| :--- | :--- | :--- | :--- |
| `true_angle_tolerance`| `0.05` | 击打点与X轴夹角阈值 | 确保真实的装甲板就在枪口指向范围内。 |
| `aim_angle_tolerance` | `0.05` | 瞄准点与X轴夹角阈值 | 确保云台不仅到位，且指向精度达标。 |

### 话题说明
*   **发布指令**：`/robot/aim` (类型：`robot_interfaces/msg/Aim`)。
*   **发布调试**：`/ballistics/debug` (类型：`robot_interfaces/msg/BallisticsDebug`)。

### 滤波参数 (Dynamic Poly-Fit)
> 注意：该滤波器已升级为基于序列 `time_offset` 的动态最小二乘拟合。

| 参数名 | 默认值 | 说明 | 调试建议 |
| :--- | :--- | :--- | :--- |
| `enable_sg_yaw / pitch`| `true` | 是否开启平滑滤波 | 建议始终开启，能极大改善角速度指令稳定性。 |
| `sg_yaw / pitch_order` | `2` | 拟合多项式阶数 | **推荐设为 4**：能更好地贴合旋转产生的正弦轨迹。 |

---

## 3. ManagerNode (生命周期管理)

该节点负责根据比赛状态（模式切换）自动激活或停用核心算法节点。

### 核心逻辑
*   **订阅话题**：`/robot/mode` (类型：`robot_interfaces/msg/Mode`)。
*   **动作**：
    *   当 `mode == 0` 时：向 `armor_detector` 和 `armor_solver` 发送 `ACTIVATE` 指令。
    *   当 `mode != 0` 时：向上述节点发送 `DEACTIVATE` 指令。
*   **参数**：
    *   `detector_name`: 识别节点的名称（默认 `armor_detector`）。
    *   `solver_name`: 预测节点的名称（默认 `armor_solver`）。

---

## 4. robot_bringup (一键启动与多机管理)

该包集成了所有配置与启动脚本，支持多机部署与零拷贝通信。

### 启动方式
使用 `robot_type` 参数切换不同机器的配置：
```bash
ros2 launch robot_bringup vision.launch.py robot_type:=sentry
```
可用的 `robot_type`: `default`, `sentry`, `infantry_3`, `infantry_4`。

---

## 5. 常见问题排查 (Troubleshooting)

### Q1: 观察到波形中有微小的“阶梯感”或不连续细节？
*   **对策**：尝试调低 `trajectory.switch_concentration`（如 `10.0`）来平滑装甲板切换。
*   **原理**：虽然已使用动态拟合，但过高的集中度会导致输入信号在切换瞬间斜率过大。

### Q2: 旋转时预瞄点偏向装甲板外侧？
*   **原因**：2 阶多项式拟合高曲率弧线时会产生径向向外的系统偏差。
*   **对策**：将 `sg_yaw_order` 设置为 **4**。

### Q3: 目标进入 TEMP_LOST 状态后，可视化停止更新？
*   **状态**：已在代码层修复。只要卡尔曼已收敛，系统会根据惯性持续外推。
*   **检查**：若仍发生，请确保 `max_lost_duration` 大于丢帧时长。

### Q4: 如何精确微调延迟补偿？
1. 观察 `/ballistics/debug` 话题。
2. 对比 `raw_yaw` 与滤波后的指令。
3. 若子弹总是落在目标旋转方向的后方：**增大 `trajectory.hit_delay_offset`**。
4. 若云台指向总是慢半拍：**增大 `trajectory.aim_delay_offset`**。

### Q5: 为什么 `success` 位跳变频繁？
*   **原因**：`true_trajectory` 使用的是物理真实的硬切换逻辑。当装甲板切换时，`true_pt` 会发生空间跳变。
*   **对策**：检查 `true_angle_tolerance` 是否太小，或者 `trajectory.dt` 是否太大（建议 `0.02 - 0.05`）。

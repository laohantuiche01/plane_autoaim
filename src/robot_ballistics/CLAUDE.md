[根目录](../../CLAUDE.md) > [src](../) > **robot_ballistics**

# robot_ballistics

## 模块职责

**弹道解算与火控判定节点**，负责将 `ArmorSolverNode` 生成的三维轨迹序列转换为云台电机可直接执行的绝对角度指令，并判定是否满足开火条件：

- 接收双轨制预测轨迹（`TargetTrajectory`，v1.1.0+ 含装甲板宽度信息）
- 对轨迹中每个点进行**抛物线弹道补偿**（考虑重力下坠，求解仰角 $\theta_{pitch}$）
- 使用 **Savitzky-Golay 动态多项式拟合**对角度序列进行平滑，并提取中心时刻的精确角速度（前馈项）
- **v1.1.0+ 改进**：
  - 动态开火容差：根据装甲板半宽和距离自适应计算（`tolerance_coefficient`）
  - 击发延时补偿：success 标志在轨迹时间序列中前移（`fire_delay`）
  - 解析法+SG融合：将 3D 速度解析转换与角度 SG 滤波融合（`use_analytical_w_yaw`、`analytical_w_yaw_alpha`）
  - 弹道可视化：终点判定改为目标距离（v1.1.0+）
- 判定物理击打点与瞄准点是否同时在动态容差范围内，生成 `success` 开火许可标志
- 将最终指令通过 `/robot/aim` 话题发布给 `robot_communication`

---

## 节点列表

### BallisticsNode

| 属性 | 内容 |
|------|------|
| 节点名称 | `ballistics_node` |
| Plugin 名称 | `robot_ballistics::BallisticsNode` |
| 可执行文件 | `ballistics_component_node`（Component） |
| 节点类型 | 普通节点（非生命周期） |

**订阅话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `armor_solver/trajectory` | `robot_interfaces/msg/TargetTrajectory` | 双轨制预测轨迹（每次收到即处理） |

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot/aim` | `robot_interfaces/msg/Aim` | 瞄准指令（最终输出，yaw/pitch/w_yaw/w_pitch/success） |
| `/ballistics/debug` | `robot_interfaces/msg/BallisticsDebug` | 调试信息（原始角度误差、开火判定结果） |
| `/ballistics/trajectory_marker` | `visualization_msgs/msg/Marker` | 抛物线弹道可视化（LINE_STRIP，绿色=允许开火，黄色=等待） |

**关键参数**

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `bullet_speed` | 25.0 | 子弹初速 (m/s) |
| `gimbal_frame` | `"gimbal_link"` | 云台坐标系帧名称 |
| `true_angle_tolerance` | 0.05 | 物理击打点偏航角容差 (rad) |
| `aim_angle_tolerance` | 0.05 | 瞄准点 Yaw/Pitch 综合容差 (rad) |
| `hit_yaw_offset` | 0.0 | 击打点偏航角补偿 (rad) |
| `hit_pitch_offset` | 0.0 | 击打点俯仰角补偿 (rad) |
| `aim_yaw_offset` | 0.00872 | 瞄准点偏航角补偿 (rad，约 0.5°) |
| `aim_pitch_offset` | -0.016 | 瞄准点俯仰角补偿 (rad，约 -0.9°) |
| `enable_sg_yaw` | false | 是否对偏航角启用 SG 滤波 |
| `sg_yaw_order` | 2 | SG 滤波多项式阶数（偏航） |
| `enable_sg_pitch` | false | 是否对俯仰角启用 SG 滤波 |
| `sg_pitch_order` | 2 | SG 滤波多项式阶数（俯仰） |
| `tolerance_coefficient` | 1.0 | 动态容差系数 (v1.1.0+) |
| `fire_delay` | 0.0 | 击发延时补偿 (s)，建议 0.01-0.05 (v1.1.0+) |
| `use_analytical_w_yaw` | false | 使用解析法+SG融合计算 yaw 前馈 (v1.1.0+) |
| `analytical_w_yaw_alpha` | 0.7 | 解析法权重，范围 [0, 1] (v1.1.0+) |

---

## 弹道计算原理

重力补偿采用抛物线方程求解：

$$z = d \cdot \tan(\theta) - \frac{g \cdot d^2}{2 v^2}(1 + \tan^2(\theta))$$

其中 $d$ 为水平投影距离，$z$ 为目标高度差，$v$ 为子弹初速。方程关于 $u=\tan(\theta)$ 为一元二次方程，直接求解。当判别式 $\Delta < 0$ 时（目标超出射程），退化为 $\arctan(z/d)$。

---

## 关键依赖

- `rclcpp`、`rclcpp_components`
- `robot_interfaces`（TargetTrajectory、Aim、BallisticsDebug）
- `robot_utils`（`unwrap_angles`、`normalize_angle`、`SavitzkyGolayFilter`）
- `Eigen3`
- `tf2_ros`、`tf2_geometry_msgs`
- `visualization_msgs`

---

## 构建

```bash
colcon build --packages-select robot_ballistics
```

本包同时导出 `ballistics_calculator` 静态库，供 `robot_auto_aim` 中的飞行时间预估使用。

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.1.0 | 2026-03-23 | 四项火控改进：动态容差、击发延时补偿、解析法+SG融合前馈、弹道可视化终点修复；TargetTrajectory 添加 armor_width |
| 1.0.0 | 2026-03-22 | 初始化文档，自动生成 |

---

## 相关文件清单

| 文件 | 说明 |
|------|------|
| `src/ballistics_node.cpp` | 节点实现（轨迹订阅、弹道解算、指令发布） |
| `src/ballistics_calculator.cpp` | 弹道计算核心（仰角求解、飞行时间计算） |
| `include/robot_ballistics/ballistics_node.hpp` | 节点类声明 |
| `include/robot_ballistics/ballistics_calculator.hpp` | BallisticsCalculator 类声明 |
| `CMakeLists.txt` | 构建配置，导出 ballistics_calculator 供外部使用 |

[根目录](../../CLAUDE.md) > [src](../) > **robot_interfaces**

# robot_interfaces

## 模块职责

项目的**自定义 ROS 2 接口定义包**，仅包含消息（`.msg`）定义，不包含任何节点或可执行文件。所有其他包均依赖本包生成的 C++ 头文件和 Python 模块。

---

## 自定义消息定义

### 核心话题消息

| 消息类型 | 文件 | 字段说明 |
|----------|------|----------|
| `Armor` | `msg/Armor.msg` | 单块装甲板：`number`(字符串)、`type`(small/large)、`distance_to_image_center`(float32)、`pose`(geometry_msgs/Pose，相机系 3D 位姿) |
| `Armors` | `msg/Armors.msg` | 装甲板列表：`header` + `Armor[]` |
| `Aim` | `msg/Aim.msg` | 瞄准指令：`header`、`yaw`(rad)、`pitch`(rad)、`w_yaw`(rad/s)、`w_pitch`(rad/s)、`target_rate`(uint8)、`target_number`(uint8)、`success`(bool) |
| `Gimbal` | `msg/Gimbal.msg` | 云台姿态：`header`、`pitch`/`roll`/`yaw`(float32，rad) |
| `Mode` | `msg/Mode.msg` | 工作模式：`mode`(uint8，1=自瞄激活)、`is_pressing`(bool) |
| `TargetState` | `msg/TargetState.msg` | UKF 滤波后目标完整状态（11 维机器人模型 / 7 维前哨站模型）：位置、速度、偏航角、角速度、旋转半径、长短轴差、高度差及其方差 |
| `TargetTrajectory` | `msg/TargetTrajectory.msg` | 双轨制预测轨迹：`header` + `true_trajectory[]`（物理击打轨迹）+ `aim_trajectory[]`（平滑瞄准轨迹） |
| `TargetTrajectoryPoint` | `msg/TargetTrajectoryPoint.msg` | 轨迹点：`time_offset`(s)、`x/y/z`(m)、`v_x/v_y/v_z`(m/s) |
| `Measurement` | `msg/Measurement.msg` | UKF 原始观测值（调试用）：`header`、`x/y/z`、`yaw` |

### 调试消息

| 消息类型 | 文件 | 说明 |
|----------|------|------|
| `DebugLight` | `msg/DebugLight.msg` | 单灯条调试信息 |
| `DebugLights` | `msg/DebugLights.msg` | 灯条列表调试信息 |
| `DebugArmor` | `msg/DebugArmor.msg` | 单装甲板调试信息 |
| `DebugArmors` | `msg/DebugArmors.msg` | 装甲板列表调试信息 |
| `BallisticsDebug` | `msg/BallisticsDebug.msg` | 弹道解算调试：`raw_yaw/pitch`、`true_angle_to_x`、`aim_angle_to_x`、`success` |

---

## 关键依赖

- `rosidl_default_generators`（构建时）
- `rosidl_default_runtime`（运行时）
- `std_msgs`（Header）
- `geometry_msgs`（Pose、Point 等）

---

## 构建

```bash
colcon build --packages-select robot_interfaces
```

注意：修改 `.msg` 文件后，必须重新构建本包，并重新构建所有依赖本包的包。

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0.0 | 2026-03-22 | 初始化文档，自动生成 |

---

## 相关文件清单

| 文件 | 说明 |
|------|------|
| `msg/Aim.msg` | 弹道输出指令 |
| `msg/Armor.msg` | 单装甲板信息 |
| `msg/Armors.msg` | 装甲板列表 |
| `msg/Gimbal.msg` | 云台姿态 |
| `msg/Mode.msg` | 工作模式 |
| `msg/TargetState.msg` | UKF 估计状态 |
| `msg/TargetTrajectory.msg` | 预测轨迹（双轨制） |
| `msg/TargetTrajectoryPoint.msg` | 轨迹点 |
| `msg/Measurement.msg` | 调试观测值 |
| `msg/BallisticsDebug.msg` | 弹道调试 |
| `msg/DebugLight.msg` / `DebugLights.msg` | 灯条调试 |
| `msg/DebugArmor.msg` / `DebugArmors.msg` | 装甲板调试 |
| `CMakeLists.txt` | 使用 rosidl_generate_interfaces 自动扫描 msg/ 目录 |

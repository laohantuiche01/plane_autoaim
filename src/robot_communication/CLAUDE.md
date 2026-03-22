[根目录](../../CLAUDE.md) > [src](../) > **robot_communication**

# robot_communication

## 模块职责

机器人串口通信网关，作为视觉算法系统与底盘主控芯片之间的**全双工桥接器**：

- **接收方向（硬件 -> ROS）**：通过 CDC 串口（`/dev/ttyACM0`，波特率 4000000）接收底盘 MCU 的云台姿态帧（ID `0x14`），解析出 Roll/Pitch/Yaw，通过 `tf2_ros::TransformBroadcaster` 实时发布 TF 变换树（`odom -> gimbal_link -> camera_optical_frame`），并发布 `/robot/gimbal`、`/robot/mode` 话题
- **发送方向（ROS -> 硬件）**：订阅 `/robot/aim`（弹道解算结果），将角度值从弧度转换为角度，打包为 `aim_data_t` 结构体后以帧 ID `0x81` 写入串口

---

## 节点列表

### RobotCommunication

| 属性 | 内容 |
|------|------|
| 节点名称 | `robot_communication` |
| 可执行文件 | `robot_communication_node` |
| 节点类型 | 独立节点（非 Component，非生命周期节点） |

**订阅话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot/aim` | `robot_interfaces/msg/Aim` | 弹道解算输出的目标瞄准角度与角速度，接收后转发串口 |

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot/gimbal` | `robot_interfaces/msg/Gimbal` | 云台当前姿态（Roll/Pitch/Yaw，单位 rad） |
| `/robot/mode` | `robot_interfaces/msg/Mode` | 底盘模式标志位（mode=1 表示自瞄激活） |
| `/robot/gimbal_transform` | `geometry_msgs/msg/TransformStamped` | 云台变换的话题形式冗余发布 |
| TF: `odom -> gimbal_link` | `tf2` 变换树 | 世界系到云台系的动态变换 |
| TF: `gimbal_link -> camera_optical_frame` | `tf2` 变换树 | 云台系到相机光学系的静态外参变换 |

**关键参数**

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `serial_name` | `/dev/ttyACM0` | CDC 串口设备路径 |
| `camera_x/y/z` | 0.0 | 相机安装位置（相对云台中心，米） |
| `camera_roll/pitch/yaw` | 0.0 | 相机安装角度（相对云台，度） |

---

## 串口协议

采用自定义帧格式，消息体使用 `__attribute__((packed))` 紧凑结构体。关键帧 ID：

| 帧 ID | 方向 | 数据结构 | 说明 |
|-------|------|----------|------|
| `0x14` | MCU -> NUC | `gimbal_and_config_data_t` | 云台姿态 + 模式 |
| `0x81` | NUC -> MCU | `aim_data_t` | 瞄准指令（Pitch/Yaw/角速度/开火许可） |
| `0x11` | MCU -> NUC | `imu_data_t` | IMU 原始数据（当前未使用） |
| `0x12` | MCU -> NUC | `tyre_speed_data_t` | 轮速里程计（当前未使用） |

串口实现依赖第三方库 `serialib`（位于 `thirdparty/serialib/`）。

---

## 关键依赖

- `rclcpp`、`tf2_ros`、`tf2`
- `robot_interfaces`（Aim、Gimbal、Mode 消息）
- `robot_utils`（角度换算工具：`rad_to_deg`、`deg_to_rad`）
- `serialib`（串口操作，本地第三方库）

---

## 构建

```bash
colcon build --packages-select robot_communication
```

注意：串口设备权限需提前配置（通常需要将用户加入 `dialout` 组）。

---

## 常见问题

- **启动时报 "Open cdc device Failed"**：检查 `/dev/ttyACM0` 是否存在、是否有读写权限，或尝试修改 `serial_name` 参数
- **TF 变换异常**：检查 `camera_x/y/z/roll/pitch/yaw` 参数是否正确设置了相机外参

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0.0 | 2026-03-22 | 初始化文档，自动生成 |

---

## 相关文件清单

| 文件 | 说明 |
|------|------|
| `include/robot_communication/robot_communication.h` | 主节点类定义及内联实现 |
| `include/robot_communication/robot_message.h` | 串口帧结构体定义（帧 ID 常量、packed 结构） |
| `include/serial_pro/` | 串口协议辅助头文件（序列化、监听、写入、校验） |
| `thirdparty/serialib/` | 第三方串口库 serialib |
| `src/robot_communication_node.cpp` | main 函数入口 |
| `CMakeLists.txt` | 构建配置，波特率 4000000 |

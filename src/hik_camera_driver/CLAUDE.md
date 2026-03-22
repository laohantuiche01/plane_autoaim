[根目录](../../CLAUDE.md) > [src](../) > **hik_camera_driver**

# hik_camera_driver

## 模块职责

海康机器人（Hikrobot）工业相机的 ROS 2 驱动包。负责：
- 通过 MVS SDK 枚举并连接工业相机（支持 GigE 和 USB3 Vision 协议）
- 以配置的帧率采集原始图像，附加硬件时间戳后发布
- 加载并发布相机内参（`CameraInfo`）
- 支持运行时动态调整曝光、增益、帧率、分辨率等参数
- 支持作为 ROS 2 Component 部署于 ComposableNodeContainer（零拷贝模式）

---

## 节点列表

### HikCameraDriver

| 属性 | 内容 |
|------|------|
| 节点名称 | `camera_node` (独立节点) / `hik_camera` (Component 部署时由 launch 指定) |
| Plugin 名称 | `HikCameraDriver` |
| 可执行文件 | `hik_camera_driver_component_node`（Component）、`hik_camera_driver_node`（独立节点） |

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `image_raw` | `sensor_msgs/msg/Image` | 原始彩色图像（BGR 格式），SensorDataQoS |
| `image_raw_info` | `sensor_msgs/msg/CameraInfo` | 相机内参与畸变系数，深度为 1 |

**无订阅话题**（纯数据源节点）

**关键参数**

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `exposure_time` | 1500.0 | 曝光时间 (μs) |
| `gain` | 15.0 | 模拟增益 (dB) |
| `frame_rate` | 250.0 | 目标帧率 (Hz) |
| `delay_ratio` | 0.0 | 时间戳延迟补偿比例 |
| `width` | 1440 | 图像宽度 (px) |
| `height` | 1080 | 图像高度 (px) |
| `offset_x` | 0 | ROI 水平偏移 |
| `offset_y` | 0 | ROI 垂直偏移 |
| `bit_depth` | `"Bits_8"` | ADC 位深 |
| `k` | `[fx,0,cx,0,fy,cy,0,0,1]` | 相机内参矩阵（9 个元素） |
| `d` | `[0,0,0,0,0]` | 畸变系数（5 个元素，k1 k2 p1 p2 k3） |

所有参数均支持运行时动态更新（通过 `ros2 param set`）。

---

## 关键依赖

- **MVS SDK**：海康机器人视觉系统 SDK，头文件路径 `/opt/MVS/include`，库文件路径 `/opt/MVS/lib/64`（x86_64）或 `/opt/MVS/lib/aarch64`（ARM）
- `rclcpp`、`rclcpp_components`
- `sensor_msgs`（Image、CameraInfo）
- `OpenCV`（图像格式转换）

---

## 构建与安装

```bash
# 需确保 MVS SDK 已安装至 /opt/MVS/
colcon build --packages-select hik_camera_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

若目标平台为 aarch64（如 NUC 或 ARM 工控机），CMake 会自动检测并切换库路径。

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0.0 | 2026-03-22 | 初始化文档，自动生成 |

---

## 相关文件清单

| 文件 | 说明 |
|------|------|
| `src/hik_camera_driver.cpp` | 核心驱动逻辑（设备枚举、SDK 调用、图像回调） |
| `src/hik_camera_driver_component_node.cpp` | Component 注册入口 |
| `src/hik_camera_driver_node.cpp` | 独立节点 main 函数 |
| `CMakeLists.txt` | 构建配置，含平台架构自动检测 |
| `package.xml` | ROS 2 包描述（依赖 opencv2、rclcpp、sensor_msgs） |

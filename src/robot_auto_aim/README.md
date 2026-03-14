# robot_auto_aim

`robot_auto_aim` 是为机器人视觉自动瞄准系统设计的 ROS 2 工具包。当前包含的 `armor_detector` 节点通过 Lifecycle 生命周期进行管理。

## 节点说明：`armor_detector`
基于原 `armor_detector` 进行重构，使用了标准的 `rclcpp_lifecycle::LifecycleNode` 生命周期管理方式进行控制，并使用了标准的 `RCLCPP_INFO` 等 ROS 2 日志工具，移除了复杂命名空间。

该节点主要用于：
1. **订阅相机图像**（支持零拷贝机制进行 Intra-Process 高效传输）。
2. **处理图像** 以检测装甲板信息并使用 PNP 求解位姿。
3. **发布结果与可视化信息** 以供后续处理和调试。

### 状态机描述（Lifecycle）

- **Unconfigured**: 初始状态，等待参数配置。
- **Configured**: 初始化内部特征提取器（Detector）、相机 TF 参数、Marker 发布器和 Debug 发布器。
- **Active**: 激活相机订阅 (`image_raw`)、参数动态调整及所有发布器。相机消息采用基于 `sensor_msgs::msg::Image::ConstSharedPtr` 的零拷贝通信进行处理。
- **Deactivated**: 停止订阅相机图像，暂停发布器，减少不必要的系统资源开销（例如：在非自瞄模式下降低网络及 CPU 使用率）。

### 话题 (Topics)

**订阅:**
* `image_raw` (`sensor_msgs/msg/Image`) - 原始图像输入（仅在 Active 状态订阅）
* `camera_info` (`sensor_msgs/msg/CameraInfo`) - 相机内参（仅在 Active 启动时订阅一次）

**发布:**
* `armor_detector/armors` (`rm_interfaces/msg/Armors`) - 识别到的装甲板位姿信息及分类
* `armor_detector/marker` (`visualization_msgs/msg/MarkerArray`) - 装甲板 3D 位姿可视化
* `armor_detector/debug_lights` (`rm_interfaces/msg/DebugLights`) - [可选] Debug 灯条信息
* `armor_detector/debug_armors` (`rm_interfaces/msg/DebugArmors`) - [可选] Debug 装甲板筛选信息
* `armor_detector/binary_img` (`sensor_msgs/msg/Image`) - [可选] Debug 二值化图像
* `armor_detector/number_img` (`sensor_msgs/msg/Image`) - [可选] Debug 数字分类图像
* `armor_detector/result_img` (`sensor_msgs/msg/Image`) - [可选] Debug 叠加渲染结果图像

### 使用说明

因为使用了 `ament_cmake_auto` 架构，您可以直接编译 `robot_auto_aim` 包。

```bash
colcon build --packages-select robot_auto_aim
```

由于它是一个 Lifecycle Node，在启动后需要通过 Lifecycle Manager 或命令工具将其转换至 `Active` 状态：
```bash
ros2 lifecycle set /armor_detector configure
ros2 lifecycle set /armor_detector activate
```

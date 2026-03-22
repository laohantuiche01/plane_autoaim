[根目录](../../CLAUDE.md) > [src](../) > **robot_bringup**

# robot_bringup

## 模块职责

**系统启动与配置管理包**，不包含任何 C++ 代码，仅提供：

- 多机型参数配置文件（YAML）
- 实车启动 launch 文件
- Rosbag 离线调试启动文件
- 相机标定启动文件

通过 `robot_type` 参数在启动时动态加载对应机型的参数表，实现多车型统一部署。

---

## 启动文件

### vision.launch.py（实车启动）

**功能**：启动完整的实车视觉自瞄流水线。

**启动顺序**：
1. 启动 Iceoryx RouDi 守护进程（零拷贝 IPC 依赖）
2. 启动 `vision_container`（ComposableNodeContainer，绑核 0-3，优先级 nice -20）：
   - `hik_camera` (HikCameraDriver)
   - `armor_detector` (ArmorDetectorNode)
   - `armor_solver` (ArmorSolverNode)
   - `ballistics_node` (BallisticsNode)
3. 启动 `robot_manager` 独立节点
4. 启动 `robot_communication` 独立节点

**启动参数**

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `robot_type` | `default` | 机型配置（default / infantry_3 / infantry_4 / sentry） |
| `use_file_log` | `true` | 是否将日志写入文件 |
| `log_path` | `~/ckyf_vision_log` | 日志根目录 |

```bash
ros2 launch robot_bringup vision.launch.py robot_type:=infantry_4
```

---

### vision_bag.launch.py（Rosbag 调试）

**功能**：用于离线 Rosbag 回放调试，不启动相机驱动和通信节点。

**启动顺序**（含延时）：
1. t=0s：启动 Iceoryx RouDi
2. t=2s：启动 vision_container（含 ArmorDetector、ArmorSolver、Ballistics，`use_sim_time:=true`）
3. t=3s：启动 `ros2 bag play --clock -l <bag_path>`（循环播放，绑核 4-5）
4. t=4s：启动 robot_manager

**必填参数**

| 参数名 | 说明 |
|--------|------|
| `bag_path` | Rosbag 目录的绝对路径（必须提供） |

```bash
ros2 launch robot_bringup vision_bag.launch.py \
  robot_type:=default \
  bag_path:=/path/to/bag
```

---

### camera_calibration.launch.py（相机标定）

参考 `/home/mijiao/ckyf_vision/docs/camera_calibration_guide.md`。

---

## 配置文件结构

```
config/
├── default/
│   └── params.yaml          # 默认参数（通用/调试用）
├── infantry_3/
│   └── params.yaml          # 步兵3号专用
├── infantry_4/
│   └── params.yaml          # 步兵4号专用
└── sentry/
    └── params.yaml          # 哨兵专用
```

`params.yaml` 中配置了所有节点的参数，通过各节点名称作为命名空间区分：
- `hik_camera.ros__parameters.*`（相机参数、内参）
- `armor_detector.ros__parameters.*`（检测阈值）
- `armor_solver.ros__parameters.*`（UKF 噪声、轨迹参数）
- `ballistics_node.ros__parameters.*`（弹道补偿参数）
- `robot_manager.ros__parameters.*`
- `robot_communication.ros__parameters.*`（串口、外参）

---

## 关键依赖

- `ament_cmake`（构建）
- Python3、`launch`、`launch_ros`（launch 文件运行时）
- 所有算法包（间接依赖，通过 launch 文件加载）

---

## 构建

```bash
colcon build --packages-select robot_bringup
```

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0.0 | 2026-03-22 | 初始化文档，自动生成 |

---

## 相关文件清单

| 文件 | 说明 |
|------|------|
| `launch/vision.launch.py` | 实车完整启动入口 |
| `launch/vision_bag.launch.py` | Rosbag 调试启动入口 |
| `launch/camera_calibration.launch.py` | 相机标定启动 |
| `launch/start_roudi.sh` | Iceoryx RouDi 启动脚本 |
| `config/default/params.yaml` | 默认全局参数 |
| `config/infantry_3/params.yaml` | 步兵3号参数 |
| `config/infantry_4/params.yaml` | 步兵4号参数 |
| `config/sentry/params.yaml` | 哨兵参数 |

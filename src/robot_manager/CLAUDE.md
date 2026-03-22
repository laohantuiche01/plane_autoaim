[根目录](../../CLAUDE.md) > [src](../) > **robot_manager**

# robot_manager

## 模块职责

**生命周期管理节点**，负责在系统启动和模式切换时自动管理 `ArmorDetectorNode` 和 `ArmorSolverNode` 的生命周期状态：

- 系统启动后延迟 2 秒，自动依次 Configure -> Activate 检测节点和求解节点
- 监听 `/robot/mode` 话题，当模式变化时动态激活（mode=1）或休眠（mode≠1）视觉流水线
- 通过 ROS 2 生命周期服务（`change_state`、`get_state`）进行状态转换，内置超时重试机制

---

## 节点列表

### ManagerNode

| 属性 | 内容 |
|------|------|
| 节点名称 | `robot_manager` |
| Plugin 名称 | `robot_manager::ManagerNode` |
| 可执行文件 | `manager_component_node` |
| 节点类型 | 普通节点（非生命周期） |

**订阅话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot/mode` | `robot_interfaces/msg/Mode` | 底盘模式指令，mode 变化时触发生命周期切换 |

**无发布话题**（通过服务调用控制其他节点）

**调用服务**

| 服务 | 类型 | 说明 |
|------|------|------|
| `armor_detector/change_state` | `lifecycle_msgs/srv/ChangeState` | 触发检测节点状态转换 |
| `armor_detector/get_state` | `lifecycle_msgs/srv/GetState` | 查询检测节点当前状态 |
| `armor_solver/change_state` | `lifecycle_msgs/srv/ChangeState` | 触发求解节点状态转换 |
| `armor_solver/get_state` | `lifecycle_msgs/srv/GetState` | 查询求解节点当前状态 |

**关键参数**

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `detector_name` | `"armor_detector"` | 检测节点名称（用于构造服务路径） |
| `solver_name` | `"armor_solver"` | 求解节点名称 |

---

## 关键依赖

- `rclcpp`、`rclcpp_components`
- `lifecycle_msgs`（ChangeState、GetState 服务，State 和 Transition 枚举）
- `robot_interfaces`（Mode 消息）

---

## 构建

```bash
colcon build --packages-select robot_manager
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
| `src/manager_node.cpp` | 节点实现（生命周期服务调用、模式回调） |
| `include/robot_manager/manager_node.hpp` | 节点类声明 |
| `CMakeLists.txt` | 构建配置 |

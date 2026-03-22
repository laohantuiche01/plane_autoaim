[根目录](../../CLAUDE.md) > [src](../) > **robot_utils**

# robot_utils

## 模块职责

项目的**公共 C++ 工具库**，不包含任何 ROS 2 节点，仅提供编译时链接的共享库 `robot_utils`。包含以下功能模块：

- **UKF（无迹卡尔曼滤波器）**：模板类，支持任意维度状态向量，含 Sigma 点生成、时间更新、测量更新、自适应噪声缩放
- **PnP 解算器**：基于 OpenCV `solvePnP` 封装，用于装甲板三维位姿估计
- **数学工具**：角度归一化（`normalize_angle`）、角度展开（`unwrap_angles`）、弧度/角度互转、Eigen 与 tf2 矩阵互转（`tf2ToEigen`）、EnemyColor 枚举
- **URL 解析器**：将 `package://` 形式的路径解析为本地文件系统绝对路径（`URLResolver::getResolvedPath`）
- **Savitzky-Golay 滤波器**：基于动态多项式最小二乘拟合，支持非等间隔时间序列，可同时输出中心点值与一阶导数（角速度）

---

## 对外接口（头文件）

| 头文件 | 主要内容 |
|--------|----------|
| `include/robot_utils/ukf.hpp` | `robot_utils::UKF<N>` 模板类（header-only） |
| `include/robot_utils/pnp_solver.hpp` | `robot_utils::PnPSolver` 类 |
| `include/robot_utils/math_utils.hpp` | 角度工具、坐标系转换（tf2/cv→Eigen）、球坐标转换（`cartesianToSpherical`/`sphericalToCartesian`）、`SphericalIdx` 枚举、`computeViewingAngle` |
| `include/robot_utils/url_resolver.hpp` | `robot_utils::URLResolver::getResolvedPath` |
| `include/robot_utils/savitzky_golay.hpp` | `robot_utils::SavitzkyGolayFilter` 类 |
| `include/robot_utils/common.hpp` | 公共类型定义 |

---

## 关键依赖

- `Eigen3`（矩阵运算，UKF 核心）
- `OpenCV`（PnP 解算）
- `tf2`（坐标系工具）
- `ament_index_cpp`（包路径解析，URLResolver 使用）

---

## 构建

```bash
colcon build --packages-select robot_utils
```

本包以共享库形式安装，其他包通过 `find_package(robot_utils REQUIRED)` 引用。

---

## UKF 使用要点

```cpp
// 实例化 11 维 UKF（机器人模型）
robot_utils::UKF<11> ukf(alpha, beta, kappa);

// 设置状态转移函数（时间推演）
ukf.setTransitionFunction([](const VectorX& x, double dt) -> VectorX { ... });

// 设置测量函数（观测映射）
ukf.setMeasurementFunction([](const VectorX& x) -> MeasureVector { ... });

// 预测步骤
ukf.predict(dt, Q);

// 更新步骤（含自适应 R）
ukf.update(measurement, R);
```

超参数：`ukf_alpha=0.001`、`ukf_beta=2.0`、`ukf_kappa=0.0`（均在 `armor_solver` 参数中配置）。

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0.0 | 2026-03-22 | 初始化文档，自动生成 |

---

## 相关文件清单

| 文件 | 说明 |
|------|------|
| `include/robot_utils/ukf.hpp` | UKF 模板类（含自适应噪声机制） |
| `include/robot_utils/pnp_solver.hpp` | PnP 解算器声明 |
| `include/robot_utils/math_utils.hpp` | 数学工具函数声明 |
| `include/robot_utils/url_resolver.hpp` | 包路径解析声明 |
| `include/robot_utils/savitzky_golay.hpp` | SG 滤波器声明 |
| `src/pnp_solver.cpp` | PnP 解算器实现 |
| `src/math_utils.cpp` | 数学工具实现 |
| `src/url_resolver.cpp` | URL 解析实现 |
| `src/savitzky_golay.cpp` | SG 滤波器实现 |
| `CMakeLists.txt` | 构建配置，导出 include 和 dependencies |

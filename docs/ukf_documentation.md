# 无迹卡尔曼滤波 (UKF) 使用文档

`robot_utils::UKF` 是一个基于模板的高性能无迹卡尔曼滤波器，专为机器人状态估计设计。它利用 Eigen 进行高效的线性代数运算，并结合 `std::function` 提供了极其灵活的、基于 Lambda 表达式的系统建模能力。

## 🌟 核心特性

1. **栈内存分配（无动态开销）**：通过模板化状态维度 (`N`) 和观测维度 (`M`)，所有内部矩阵运算均在栈上完成，确保滤波循环过程中零动态内存分配，极大提升运行效率。
2. **灵活的系统建模**：状态转移函数 $f(x)$ 和观测函数 $h(x)$ 通过 `std::function` 传入。你可以利用 C++ Lambda 表达式轻松切换匀速 (CV)、匀加速 (CA) 或恒定转率匀速 (CTRV) 等各种运动模型。
3. **高级角度处理（Angle Wrapping）**：内置对状态和观测残差归一化的支持（例如将角度差值限制在 $[-\pi, \pi]$），直接解决机器人领域常见的欧拉角突变导致的平均值计算错误问题。
4. **异常值剔除（马氏距离门限）**：内置基于卡方分布的马氏距离校验，能够自动拒绝离谱的传感器观测值（如装甲板误识别、激光雷达噪点等），防止滤波器发散。
5. **自适应观测噪声 (Sage-Husa)**：滤波器能够根据残差新息（Innovation）动态估计当前的观测协方差矩阵 $R$，自动适应传感器精度的变化（如运动模糊导致的相机精度下降）。
6. **衰减记忆自适应 (Fading Memory)**：支持在预测步中按比例膨胀预测协方差 $P$，防止滤波器产生“过度自信”，从而能够更好地追踪高机动目标（如疯狂走位、小陀螺）。

---

## 🚀 基础使用示例

以下示例演示如何使用 UKF 追踪一个 2D 位置和速度（状态维度 `N=4`）：

```cpp
#include "robot_utils/ukf.hpp"
#include <iostream>

int main() {
    // 1. 定义一个 4 维状态向量的 UKF: [x, y, v_x, v_y]
    robot_utils::UKF<4> ukf;

    // 2. 初始化状态 (x0) 和协方差 (P0)
    Eigen::Vector4d x0 = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity();
    ukf.init(x0, P0);

    // 3. 定义过程噪声协方差 (Q)
    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity() * 0.1;

    // 4. 预测步
    // 使用 Lambda 表达式定义状态转移函数 f(x)
    double dt = 0.01;
    auto f_func = [dt](const Eigen::Vector4d& x) {
        Eigen::Vector4d x_next = x;
        x_next(0) += x(2) * dt; // x = x + v_x * dt
        x_next(1) += x(3) * dt; // y = y + v_y * dt
        return x_next;
    };
    
    ukf.predict(f_func, Q);

    // 5. 更新步
    // 假设我们只测量位置 (x, y)，观测维度 M = 2
    Eigen::Vector2d z(1.5, 2.5); // 传感器测量值
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.05; // 观测噪声
    
    // 定义观测函数 h(x)
    auto h_func = [](const Eigen::Vector4d& x) {
        return Eigen::Vector2d(x(0), x(1)); 
    };

    // 执行更新
    ukf.update<2>(z, h_func, R);

    // 6. 获取估计结果
    std::cout << "估计状态:\n" << ukf.getState() << std::endl;

    return 0;
}
```

---

## 🛠 高级功能说明

### 1. 角度环绕处理 (Angle Wrapping)

在进行云台追踪时，经常会遇到 Yaw 或 Pitch 角度。标准的算术平均会导致 359° 和 1° 的平均值变为 180°。通过传入归一化函数可以解决此问题：

```cpp
auto normalize_angle_diff = [](Eigen::Vector4d& diff) {
    // 假设 diff(2) 是偏航角 (Yaw) 的偏差
    while (diff(2) > M_PI) diff(2) -= 2.0 * M_PI;
    while (diff(2) < -M_PI) diff(2) += 2.0 * M_PI;
};

// 在预测步中使用
ukf.predict(f_func, Q, normalize_angle_diff);

// 在更新步中使用（假设观测值也包含角度）
ukf.update<3>(z, h_func, R, normalize_meas_angle_diff, normalize_angle_diff);
```

### 2. 马氏距离剔除 (Gating)

通过设置卡方阈值防止离谱的观测数据毁掉滤波器。

```cpp
// 对于 M=3 (3个自由度的观测)，99% 置信度阈值约为 11.34
double mahalanobis_thresh = 11.34;

bool accepted = ukf.update<3>(z, h_func, R, nullptr, nullptr, mahalanobis_thresh);

if (!accepted) {
    // 观测值在统计学上是不可能的，已被忽略。
    // 状态 x_ 将保持为预测值。
    std::cout << "观测被拒绝！" << std::endl;
}
```

### 3. 衰减记忆自适应 (追踪高机动目标)

如果滤波器过于收敛（协方差 $P$ 变得极小），面对敌方突然加速或变向时会反应迟钝。使用 `fading_factor`（通常在 1.01 到 1.05 之间）可以使滤波器保持灵敏。

```cpp
double fading_factor = 1.02; // 每次预测将 P 膨胀 2%
ukf.predict(f_func, Q, nullptr, fading_factor);
```

### 4. Sage-Husa 观测噪声自适应

当环境导致传感器精度波动时，让 UKF 根据残差实时推断并更新观测协方差 $R$。

```cpp
// 初始的 R 阵猜测
Eigen::Matrix3d R_adaptive = Eigen::Matrix3d::Identity() * 0.1;

// 学习率 (alpha) 决定 R 适应新观测的速度。
// 较高值（如 0.3）适应快；较低值（如 0.05）更平滑。
double learning_rate = 0.2;

// R_adaptive 矩阵会在函数调用后在内部被自动更新！
ukf.updateAdaptive<3>(z, h_func, R_adaptive, learning_rate);
```

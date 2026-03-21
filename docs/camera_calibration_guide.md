# 相机标定教程 (Camera Calibration Guide)

为了保证视觉解算（PnP）的精度，必须对每台相机的内参（Intrinsic）和畸变系数（Distortion）进行精确标定。本工作空间提供了一键化的标定启动脚本。

## 1. 准备工作

### 1.1 安装依赖包
确保系统已安装 ROS 2 标定工具包：
```bash
sudo apt update
sudo apt install ros-humble-camera-calibration
```

### 1.2 准备标定板
- 使用打印机打印一张棋盘格标定板（推荐使用 A4 纸或粘贴在平整的硬板上）。
- 确认标定板的**内部角点数**（例如 12x9 个格子的标定板，其内部角点为 `11x8`）。
- 确认每个格子的**物理边长**（单位：米，例如 15mm 对应 `0.015`）。

---

## 2. 标定流程

### 2.1 启动标定程序
在终端执行以下命令（根据实际标定板调整参数）：

```bash
source .envrc
ros2 launch robot_bringup camera_calibration.launch.py \
    size:=11x8 \
    square:=0.015 \
    exposure_time:=2500.0
```

**关键参数说明：**
- `size`: 内部角点个数（列x行）。
- `square`: 单个格子边长（单位：米）。
- `exposure_time`: 曝光时间（单位：微秒）。标定时建议调低曝光，使黑白格边缘清晰，防止拖影和过曝。

### 2.2 采集样本
标定界面启动后，您将看到一个视频窗口。
- 拿着标定板在相机视野内移动。
- **采集准则**：
    - **X (左/右)**：使标定板涵盖图像的左缘和右缘。
    - **Y (上/下)**：使标定板涵盖图像的上缘和下缘。
    - **Size (远/近)**：使标定板充满画面或在远处采集。
    - **Skew (倾斜)**：侧向倾斜标定板，使角点产生透视变形。
- 当侧边栏的 **X, Y, Size, Skew** 四个进度条都变成**绿色**时，说明样本采集充分。

### 2.3 计算与保存
1. 点击界面上的 **CALIBRATE** 按钮。
2. 程序会卡住一段时间进行优化计算（通常需 10-60 秒）。
3. 计算完成后，终端会输出内参矩阵 `K` 和畸变系数 `D`。
4. 点击界面上的 **SAVE** 按钮。
5. 标定结果默认保存在 `/tmp/calibrationdata.tar.gz` 中。

---

## 3. 应用标定结果

### 3.1 提取参数
解压 `/tmp/calibrationdata.tar.gz`，找到内部的 `ost.yaml` 或查看终端输出的 `Camera calibration parameters`。

### 3.2 更新配置文件
将获取到的参数填入 `src/robot_bringup/config/<robot_type>/params.yaml` 中对应的 `k` 和 `d` 字段：

```yaml
hik_camera:
  ros__parameters:
    k: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    d: [k1, k2, p1, p2, k3]
```

### 3.3 验证
重新启动正常的视觉程序 `ros2 launch robot_bringup vision.launch.py`，观察 `debug_image` 中的重投影预测框是否与真实的装甲板边缘对齐良好。

---

## 4. 常见问题排查

- **无法识别格子**：检查曝光时间是否过高导致边缘模糊，或环境光线太暗。
- **CALIBRATE 按钮灰色**：进度条未变绿，请继续增加标定板的角度和覆盖范围。
- **重投影误差过大**：确保标定板是绝对平整的，标定时不要让纸张弯曲。建议重投影误差 (DDS 终端输出) 应小于 **0.5 像素**。

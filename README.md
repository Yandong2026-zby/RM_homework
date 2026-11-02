# PnP_RM_Cpp：装甲板识别与位姿估计

基于海康工业相机与 OpenCV，实现 RoboMaster 小装甲板的识别、PnP 位姿解算，
并使用 Kalman 滤波进行跟踪与丢失预测。

## 功能

- 海康相机实时采集（固定 25 fps、5 ms 曝光）
- 棋盘格相机标定，输出真实内参和畸变系数
- HSV 双灯条识别与装甲板配对
- 基于标定内参的 solvePnP 位姿解算
- 匀速运动模型 Kalman 跟踪，丢失后可预测续帧
- GUI 同时显示实测位姿（红绿蓝轴）与预测位姿（黄色轴）
- 支持离线图片/视频回放（`--src`），便于无相机调试

## 环境依赖

- Ubuntu 22.04
- CMake ≥ 3.10，g++（C++17）
- OpenCV 4.5.4
- 海康 MVS SDK（默认安装路径 `/opt/MVS`）

## 目录结构

```
.
├── CMakeLists.txt
├── main.cpp              # 离线单图 PnP 调试
├── camera_grab.cpp       # 相机取流
├── calibration.cpp       # 棋盘格标定
├── capture_armor.cpp     # 实时识别 + PnP + Kalman
├── pose_tracker.hpp      # Kalman 滤波器与坐标轴绘制
├── camera_param.yaml     # 标定结果（内参 + 畸变 + 标定分辨率）
├── calib/                # 标定照片（已 gitignore）
└── test/                 # 测试图片/视频（已 gitignore）
```

## 编译

```bash
mkdir -p build && cd build
cmake ..
make -j4
```

生成 4 个可执行文件：`main`、`grab`、`capture`、`calibration`。

## 相机标定

1. 打印 9×6 内角点、方格边长 25 mm 的棋盘格
2. 从不同角度拍摄 15–25 张，放到 `calib/`
3. 在项目根目录运行：

```bash
./build/calibration
```

输出 `camera_param.yaml`。本项目实测重投影误差 **RMS 0.71 px**。

## 使用方法

所有命令都在项目根目录执行（需要读取 `camera_param.yaml`）。

```bash
# 离线单张图片
./build/main test/Image_xxx.bmp

# 实时相机
./build/capture

# 离线图片或视频回放（不连接相机）
./build/capture --src test/kalman_demo.avi
```

运行时按键：`s` 保存、`a` 自动拍摄、`m` 显示 mask、`[` / `]` 调曝光、`q` 退出。

## 算法流程

1. **颜色分割**：BGR → HSV，红色分两段阈值后合并，形态学闭运算
2. **灯条提取**：轮廓外接矩形，保留竖直细长（长宽比 ≥ 2）的灯条
3. **灯条配对**：高度差、Y 轴差、间距/高度比筛选，取离画面中心最近的一对
4. **角点**：两根灯条的外端点，顺序为左上、右上、右下、左下
5. **位姿解算**：`solvePnP`（IPPE）结合标定内参，装甲板物理尺寸 126.5 × 33 mm
6. **Kalman 跟踪**：状态 `[x,y,z,vx,vy,vz]`，匀速模型；位置和姿态各一个滤波器
7. **预测**：检测丢失时继续 `predict()`，最多续 15 帧；GUI 用黄色轴显示预测位姿

## 关键参数

| 参数 | 位置 |
|---|---|
| HSV 红色阈值 | `capture_armor.cpp` 的 `LOW_RED_*` / `HIGH_RED_*` |
| 装甲板物理尺寸 | `capture_armor.cpp` 的 `kArmorWorld` |
| Kalman Q / R | `pose_tracker.hpp` 的 `pos_{}` / `rot_{}` |
| 采集帧率、曝光 | `capture_armor.cpp` 的 `HikCamera::Open` |

## 实验结果

- 相机标定重投影误差：0.71 px
- 离线静态图 PnP 输出稳定，重复帧间波动 < 1 mm
- 合成视频（目标匀速移动 + 丢失 15 帧）：预测与重新捕获误差约 1–2 mm
- 真实相机视频（50 s，421 帧）：识别稳定，单次短暂丢失后成功预测并重新捕获

## 已知限制

- 仅支持单块装甲板，假设四个角点共面
- 灯条角点取的是外接矩形角点，装甲板大角度滚转时精度下降
- 姿态直接对旋转向量做滤波，角度接近 180° 时可能出现跳变
- 曝光过长时运动模糊会影响识别，建议 ≤ 10 ms

## 参考

- OpenCV `solvePnP` / `KalmanFilter` 官方文档
- PnP-solver: https://github.com/Keith-iise/Pnp-solver
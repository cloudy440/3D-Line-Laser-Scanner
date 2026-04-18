<div align="center">

# 🔬 3D Line Laser Scanner

**基于线激光三角测量原理的桌面级 3D 扫描系统**

通过步进电机驱动竖直线激光绕扫描轴旋转，目标物静止放置，结合 USB 摄像头与激光三角测量，实时重建物体三维点云。

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue?logo=c%2B%2B)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-%E2%89%A53.15-064F8C?logo=cmake)](https://cmake.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-%E2%89%A54.0-5C3EE8?logo=opencv)](https://opencv.org/)
[![PCL](https://img.shields.io/badge/PCL-%E2%89%A51.12-orange)](https://pointclouds.org/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

</div>

---

## 📖 目录

- [✨ 功能特性](#-功能特性)
- [🏗️ 系统架构](#️-系统架构)
- [🔩 硬件组成](#-硬件组成)
- [📦 软件依赖](#-软件依赖)
- [📁 目录结构](#-目录结构)
- [🔨 构建方法](#-构建方法)
- [📐 标定流程](#-标定流程)
- [🚀 使用方法](#-使用方法)
- [⚙️ 配置文件说明](#️-配置文件说明)
- [☁️ 点云查看](#️-点云查看)
- [📡 串口协议简介](#-串口协议简介)
- [📄 开源协议](#-开源协议)

---

## ✨ 功能特性

| 功能 | 说明 |
|------|------|
| 🎯 **线激光中心提取** | 绿色通道重心法（Gravity Center Method），支持亚像素精度，可扩展为 Steger 算法 |
| 📐 **线面交会 3D 重建** | 基于相机内参、外参与动态激光平面方程，在相机坐标系下还原 3D 点 |
| 🔧 **全流程标定工具** | 棋盘格内参标定 + 激光平面外参标定，一键写入 `config.yaml` |
| 💻 **硬件 / 仿真双模式** | `--simulate` 标志可脱离硬件在纯软件环境中调试 |
| 🖥️ **实时双视图显示** | OpenCV 2D 激光标记窗口 + PCL 3D 点云实时生长（可选） |
| 💾 **PCD 点云保存** | 扫描结束后自动保存为标准 `.pcd` 文件 |

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────┐
│                     主机 (PC/Linux)                   │
│                                                     │
│  ┌──────────┐   ┌──────────────┐   ┌─────────────┐ │
│  │  Serial  │   │    Camera    │   │   Viewer    │ │
│  │  Comm    │   │   Capture    │   │  (2D + 3D)  │ │
│  └────┬─────┘   └──────┬───────┘   └──────┬──────┘ │
│       │                │                  │         │
│  ┌────▼────────────────▼──────────────────▼──────┐  │
│  │               main.cpp (扫描主循环)             │  │
│  │  1. 线激光臂旋转一步（目标物静止放置）           │  │
│  │  2. 获取去畸变帧                               │  │
│  │  3. LaserExtractor → 2D 亚像素中心点            │  │
│  │  4. Reconstructor  → 3D 点云切片               │  │
│  │  5. Viewer 实时渲染 & 累积                     │  │
│  └────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
         │ 串口 (115200 baud)
┌────────▼────────┐
│   STM32 控制板   │
│  步进电机驱动    │
│  激光开关控制    │
└─────────────────┘
```

**核心数学模型（线面交会）：**

$$
\text{射线方程:} \quad P_c = \lambda \cdot K^{-1} \cdot [u,\ v,\ 1]^T
$$
$$
\text{动态平面:} \quad \pi_C(\theta) = (T_{CB} \cdot T_{BR}(\theta))^{-T} \cdot \pi_R
$$
$$
\text{交会求解:} \quad \lambda = -d_c \;/\; (n_c^T \cdot K^{-1} \cdot p)
$$

---

## 🔩 硬件组成

| 组件 | 规格 / 说明 |
|------|------------|
| 📷 USB 摄像头 | 640×480，支持关闭自动曝光（曝光固定有助于激光提取） |
| 🔴 线激光器 | 532 nm 绿色线激光，沿目标物体垂直投射 |
| 🎛️ STM32 控制板 | 通过串口接收步进指令，控制激光臂旋转与激光开关 |
| ⚙️ 步进电机 + 旋转臂 | 激光器安装于旋转臂上，默认步进角 0.1125°，扫描半角 60°（可在配置中调整） |

---

## 📦 软件依赖

| 依赖 | 版本要求 | 用途 |
|------|----------|------|
| [OpenCV](https://opencv.org/) | ≥ 4.0 | 图像采集、去畸变、标定、显示 |
| [Eigen3](https://eigen.tuxfamily.org/) | ≥ 3.3 | 矩阵运算、坐标变换 |
| [PCL](https://pointclouds.org/) | ≥ 1.12 *(可选)* | 3D 点云实时可视化与 PCD 保存 |
| [Open3D](http://www.open3d.org/) | ≥ 0.17 *(Python)* | 离线点云查看脚本 |
| CMake | ≥ 3.15 | 构建系统 |
| C++ 编译器 | 支持 C++17 | `std::filesystem` 等特性 |

> [!NOTE]
> PCL 为**可选**依赖。若未安装 PCL，系统会自动禁用 3D 实时窗口，点云仍可通过 Python 脚本查看。

---

## 📁 目录结构

```
3D-Line-Laser-Scanner/
├── include/
│   ├── algorithm/
│   │   ├── laser_extractor.h    # 激光条纹中心提取接口
│   │   └── reconstructor.h      # 线面交会 3D 重建接口
│   ├── hardware/
│   │   ├── camera_capture.h     # USB 摄像头采集与去畸变接口
│   │   └── serial_comm.h        # STM32 串口通信协议接口
│   └── ui/
│       └── viewer.h             # 2D/3D 可视化接口
├── src/
│   ├── algorithm/
│   │   ├── laser_extractor.cpp
│   │   └── reconstructor.cpp
│   ├── hardware/
│   │   ├── camera_capture.cpp
│   │   └── serial_comm.cpp
│   ├── ui/
│   │   └── viewer.cpp
│   └── main.cpp                 # 扫描主程序入口
├── tools/
│   └── calibrator_main.cpp      # 独立标定工具
├── pcd_viewer.py                # Python 点云可视化脚本
└── config.yaml                  # 扫描参数与标定结果（运行前创建）
```

---

## 🔨 构建方法

```bash
# 克隆仓库
git clone https://github.com/cloudy440/3D-Line-Laser-Scanner.git
cd 3D-Line-Laser-Scanner

# 创建构建目录并编译
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel
```

编译成功后将生成两个可执行文件：

| 可执行文件 | 说明 |
|-----------|------|
| `LineLaserScanner3D` | 主扫描程序 |
| `CalibratorTool` | 标定工具 |

---

## 📐 标定流程

> [!IMPORTANT]
> 首次扫描前，需依次完成**内参标定**和**外参标定**。

### 第一步：采集内参标定图像（模式 A）

```bash
./CalibratorTool capture --camera 1 --outdir ./calib_images
```

- 按 <kbd>Space</kbd> 保存当前帧
- 建议从不同角度拍摄 **15–20 张**棋盘格图像
- 按 <kbd>Q</kbd> / <kbd>ESC</kbd> 退出

### 第二步：计算相机内参（模式 B）

```bash
./CalibratorTool calibrate \
    --imgdir ./calib_images \
    --board 9x6 \
    --square 25.0
```

| 参数 | 说明 |
|------|------|
| `--board` | 棋盘格内角点数（列×行） |
| `--square` | 单格边长（mm） |

结果写入 `config.yaml`（`Camera_Matrix`、`Distortion_Coeffs`）。

### 第三步：采集外参标定图像（模式 C）

```bash
./CalibratorTool excapture --camera 1 --outdir ./excalib_images
```

将棋盘格放置在激光平面内，开启激光线后采集。

### 第四步：计算外参与激光平面（模式 D）

```bash
./CalibratorTool excalib \
    --imgdir ./excalib_images \
    --board 9x6 \
    --square 25.0 \
    --dist 150
```

| 参数 | 说明 |
|------|------|
| `--dist` | 棋盘格到转台中心的距离（mm） |

结果更新 `config.yaml`（`T_CB`、`pi_R`）。

---

## 🚀 使用方法

### 硬件模式（连接 STM32）

```bash
./LineLaserScanner3D
```

使用 `config.yaml` 中的默认串口和摄像头参数。

### 指定配置文件

```bash
./LineLaserScanner3D --config /path/to/my_config.yaml
```

### 仿真模式（无硬件调试）

```bash
./LineLaserScanner3D --simulate
```

脱离串口连接，以固定步角递增模拟电机旋转，用于算法调试。

**扫描流程：**

```
程序启动 → 移动到扫描起点 → 逐步旋转采集 → 实时重建显示
    → 扫描完成复位 → 保存 scan_result.pcd → 按任意键退出
```

---

## ⚙️ 配置文件说明

`config.yaml` 示例（标定工具会自动生成大部分参数）：

```yaml
# ── 串口与摄像头 ─────────────────────────────────────
Serial_Port:     "COM10"    # Windows: COMx  |  Linux: /dev/ttyUSBx
Serial_BaudRate: 115200
Camera_Index:    1

# ── 扫描参数 ──────────────────────────────────────────
StepAngle_Deg:   0.1125     # 每步旋转角度（°）
Scan_Half_Angle: 60.0       # 扫描半角（°），范围 [-θ, +θ]

# ── 相机内参（由标定工具写入）────────────────────────
Camera_Matrix: !!opencv-matrix
  rows: 3
  cols: 3
  data: [ ... ]

Distortion_Coeffs: !!opencv-matrix
  rows: 1
  cols: 5
  data: [ ... ]

# ── 外参与激光平面（由标定工具写入）──────────────────
T_CB: !!opencv-matrix      # 基座 → 相机齐次变换（4×4）
  rows: 4
  cols: 4
  data: [ ... ]

LaserPlane_R: !!opencv-matrix  # 转子坐标系激光平面 [nx, ny, nz, d]
  rows: 1
  cols: 4
  data: [ ... ]
```

---

## ☁️ 点云查看

扫描完成后，使用 Python 脚本可视化 `.pcd` 文件：

```bash
pip install open3d
python pcd_viewer.py
```

> 默认读取当前目录下的 `scan_result.pcd`。如需指定路径，修改脚本中的文件名即可。

---

## 📡 串口协议简介

主机与 STM32 通过自定义帧格式通信：

**帧结构：**

| 字段 | 大小 | 值 / 说明 |
|------|------|-----------|
| `FRAME_HEAD` | 2 字节 | `0xAA 0xBB` |
| `CMD` | 1 字节 | 指令类型（见下表） |
| `DATA` | 4 字节 | `float` 数据（步数 / 角度） |
| `CRC16` | 2 字节 | 校验码 |
| `FRAME_TAIL` | 2 字节 | `0xCC 0xDD` |

**指令集：**

| 指令 | 值 | 说明 |
|------|----|------|
| `CMD_STEP` | `0x01` | 单步进 |
| `CMD_MOVE` | `0x05` | 多步移动 |
| `CMD_LASER_ON` | `0x03` | 激光开 |
| `CMD_LASER_OFF` | `0x04` | 激光关 |

---

## 📄 开源协议

本项目基于 [MIT License](LICENSE) 开源，允许自由使用、修改与分发，保留原始版权声明即可。

```
MIT License

Copyright (c) 2025 cloudy440

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

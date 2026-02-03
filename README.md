
# OAKChina-vio (Windows)

视觉惯性里程计（VIO）示例项目（Windows x64）。
本仓库包含两个示例程序：

* `oakchina_vio_demo`：显示图像并输出/打印位姿等信息（基础示例，参考https://gitee.com/oakchina/oakchina-vio.git）
* `oakchina_vio_traj_demo`：在显示图像的同时，实时绘制并交互查看 3D 轨迹（增强示例）

---

## 📋 目录

1. [支持平台](#-支持平台)
2. [前置准备](#-前置准备)

   * [1. 安装驱动](#1-安装驱动)
   * [2. 安装 OpenCV](#2-安装-opencv)
3. [本地构建](#-本地构建)
4. [运行示例程序](#-运行示例程序)

   * [运行 oakchina_vio_demo](#运行-oakchina_vio_demo)
   * [运行 oakchina_vio_traj_demo](#运行-oakchina_vio_traj_demo)
5. [交互说明（traj demo）](#-交互说明traj-demo)
6. [常见问题](#-常见问题)

---

## 🖥️ 支持平台

| 环境       | 版本                        |
| -------- | ------------------------- |
| **操作系统** | Windows x64               |
| **编译器**  | Visual Studio 2022 (MSVC) |
| **构建工具** | CMake                     |

---

## 🔧 前置准备

### 1. 安装驱动

以**管理员权限**运行 `driver/` 目录下的 **`zadig-2.7.exe`**：

1. 连接设备到电脑（建议 USB 3.0 直连，避免 HUB）。
2. 在 Zadig 中打开：`Options -> List All Devices`
3. 在下拉列表中选择 **oakchina-vio-kit** 对应设备
4. 点击 **Install Driver** 安装驱动
5. 安装完成后，在“设备管理器”中应能看到对应设备

> 若安装失败：尝试更换 USB 口/线缆，或确保没有其他软件占用设备。

---

### 2. 安装 OpenCV

1. 下载并安装 OpenCV（建议 OpenCV 4.x）：

   * OpenCV Releases：`https://opencv.org/releases/`

2. 配置环境变量（运行时需要）
   将 OpenCV 的 `bin` 目录加入系统 `PATH`，例如（以你的 OpenCV 安装路径为准）：

   * `D:\xxx\opencv\build\x64\vc16\bin`

> 如果未配置 PATH，运行时可能会提示缺少 `opencv_world*.dll` 或相关 DLL。

---

## 💻 本地构建

### 1. 获取源代码（示例）

如果你从远端克隆（以你的仓库地址为准）：

```bash
git clone https://github.com/JYZ0117/oakvio-windows_with_traj_demo.git
cd .\oakchina-vio-windows_with_traj_demo\oakchina-vio-windows
```

### 2. 使用 CMake 构建

在项目根目录创建 `build` 目录并编译：

```bash
mkdir build
cd build
cmake -DCMAKE_GENERATOR_PLATFORM=x64 ..
cmake --build . --config Release
```

编译成功后可在如下目录找到可执行文件（一般为）：

* `build\Release\oakchina_vio_demo.exe`
* `build\Release\oakchina_vio_traj_demo.exe`

---

## 🚀 运行示例程序

> **重要：运行时必须能找到 SDK 的动态库**（例如 `carina_vio.dll`）。
> 建议将 `exe` 复制到与 `carina_vio.dll` 同一目录（通常是 `lib/`），或将 DLL 所在目录加入 `PATH`。

### 运行 oakchina_vio_demo

将 `oakchina_vio_demo.exe` 复制到动态库同一目录后运行：

```bash
.\oakchina_vio_demo ..\custom_config.yaml ..\database.bin
```

---

### 运行 oakchina_vio_traj_demo

将 `oakchina_vio_traj_demo.exe` 复制到动态库同一目录后运行：

```bash
.\oakchina_vio_traj_demo ..\custom_config.yaml ..\database.bin
```

可选第三参数：是否显示图像窗口 `image_show=0/1`（默认 1）：

```bash
.\oakchina_vio_traj_demo ..\custom_config.yaml ..\database.bin 1
```

---

## 🕹️ 交互说明（traj demo）

`trajectory_3d` 窗口支持基础交互：

**鼠标**

* 左键拖拽：旋转视角（yaw/pitch）
* 右键拖拽：平移视图（pan）
* 鼠标滚轮：缩放（zoom）

**键盘**

* `r`：重置视角
* `c`：清空轨迹
* `f`：切换“跟随最新点”
* ``：保存轨迹到 `trajectory.csv`（格式：`ts,x,y,z`）
* `q` / `ESC`：退出程序

---

## ❓ 常见问题

### 1) 运行时报 “找不到 carina_vio.dll”

原因：运行目录或 `PATH` 中找不到 SDK DLL。
解决：

* 把 exe 复制到 `carina_vio.dll` 同目录；或
* 将 DLL 所在目录加入 `PATH` 环境变量。（例：set PATH=%PATH%;D:\research\...\oakchina-vio-windows\lib #换成自己的lib路径）

---

### 2) 运行时报 “缺少 opencv_world*.dll”

原因：未将 OpenCV 的 `bin` 目录加入 `PATH`。
解决：把 OpenCV 的 `...\x64\vc16\bin` 加入 `PATH`。

---

### 3) 运行后无输出/无图像

常见原因：

* 设备未识别/驱动未安装成功
* USB 连接不稳定（建议 USB 3.0 直连）
* 被其他程序占用
* 配置文件路径不正确（建议用绝对路径验证）


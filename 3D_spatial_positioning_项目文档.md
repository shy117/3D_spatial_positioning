# 3D_spatial_positioning 项目文档

## 项目定位

`3D_spatial_positioning` 是一个基于双目摄像机的三维空间定位系统。项目使用 Qt Widgets 提供界面，OpenCV 完成双目校正、立体匹配、视差滤波和三维重投影，PCL/VTK 在完整构建中负责点云保存与显示。

当前项目已整理为一份跨平台源码：Ubuntu 和 Windows 共用 `3D_spatial_positioning/` 目录下的 C++/Qt 源码，通过同一个 qmake 工程文件按平台选择依赖和功能。

## 当前目录结构

```text
3D_spatial_positioning/
├─ 3D_spatial_positioning/              # 唯一源码目录
│  ├─ 3D_spatial_positioning.pro        # 跨平台 qmake 工程
│  ├─ main.cpp
│  ├─ mainwindow.h/.cpp/.ui
│  ├─ method.h/.cpp
│  ├─ stereoconfig.h
│  └─ cloudviewerthread.h/.cpp          # 仅 HAVE_PCL 构建使用
├─ 测试场景/                              # 左右目测试图片
├─ assets/                              # README 截图资源
├─ scripts/
│  ├─ build_windows_qt6_mingw.ps1       # Windows 一键构建脚本
│  └─ deploy_windows_runtime.ps1        # Windows 运行库部署脚本
├─ README.md                            # 原 Ubuntu 环境记录
└─ 3D_spatial_positioning_项目文档.md
```

## 核心功能流程

1. 启动程序，读取 `StereoConfig` 中的双目标定参数。
2. 调用 `stereoRectify()` 和 `initUndistortRectifyMap()` 生成左右相机校正映射和 Q 重投影矩阵。
3. 从双目相机获取左右图，或从 `测试场景/` 手动选择左右测试图。
4. 将左右图转灰度，进行畸变校正和立体校正。
5. 使用 `cv::StereoSGBM` 计算视差。
6. 使用 `cv::ximgproc::createRightMatcher()` 和 `createDisparityWLSFilter()` 生成滤波视差图。
7. 使用 `cv::reprojectImageTo3D()` 将视差图重投影到三维坐标。
8. 点击图像测距，输出像素坐标、三维坐标、视差值和拟合距离。
9. 在启用 PCL 的构建中，将三维点转换为 `pcl::PointCloud<pcl::PointXYZRGBA>`，保存并显示 `.pcd` 点云。

## 跨平台整理结果

### 统一源码

此前 Windows 版在 `windows_port/src` 中维护了一份源码副本。现在已删除该副本，Windows 和 Ubuntu 都使用：

```text
3D_spatial_positioning/
```

这样算法、界面和后续修复都只需要改一处。

### 统一工程文件

工程入口统一为：

```text
3D_spatial_positioning/3D_spatial_positioning.pro
```

该 `.pro` 按平台分支：

- `unix:!android`：Ubuntu 构建，启用 `HAVE_PCL`，链接 OpenCV、PCL、VTK、Boost。
- `win32`：Windows 构建，使用本机 Qt 6 MinGW 和已有 OpenCV 包，当前默认不启用 PCL/VTK，点云通过 ASCII PLY 文件导出。

### 条件编译

点云相关代码通过 `HAVE_PCL` 控制：

- Ubuntu 完整环境定义 `HAVE_PCL`，保留点云生成、保存和显示。
- 当前 Windows 环境未找到 PCL/VTK，未定义 `HAVE_PCL`，但仍可基于 OpenCV 三维重投影结果导出 ASCII PLY 点云文件。

## Ubuntu 构建

原始环境：

- Ubuntu 18.04
- Qt 5.12.12
- OpenCV 4.4.0，包含 `opencv_contrib/ximgproc`
- PCL 1.8
- VTK 6.3
- Boost
- qmake

构建命令示例：

```bash
cd 3D_spatial_positioning
mkdir -p build/ubuntu-qmake
cd build/ubuntu-qmake
qmake ../../3D_spatial_positioning/3D_spatial_positioning.pro
make -j4
```

Ubuntu 分支仍使用原项目中的典型依赖路径：

```text
/usr/local/include/opencv4
/usr/local/lib/libopencv_*.so
/usr/include/vtk-6.3
/usr/lib/x86_64-linux-gnu/libvtk*.so
/usr/include/pcl-1.8
/usr/lib/x86_64-linux-gnu/libpcl_*.so
/usr/include/boost
/usr/lib/x86_64-linux-gnu/libboost_*.so
```

如本机依赖路径不同，修改 `.pro` 的 `unix:!android` 分支即可。

## Windows 构建

当前 Windows 已验证环境：

- Windows 10/11 x64
- Qt `D:\QT\6.8.0\mingw_64`
- MinGW `D:\QT\Tools\mingw1310_64`
- OpenCV 包：`E:\Workspace\MyGit\videoCapture\videoCapture\3rdparty\opencv`
- OpenCV 运行库：`E:\Workspace\MyGit\videoCapture\videoCapture\3rdparty\bin`

一键构建：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build_windows_qt6_mingw.ps1
```

输出程序：

```text
build/windows-qt6-mingw/bin/3D_spatial_positioning.exe
```

Windows 点云输出：

```text
imgs/pcd/PointCloud_YYYYMMDD_HHMMSS.ply
```

PLY 文件包含 `x y z red green blue` 顶点属性，可用 CloudCompare、MeshLab 或 Blender 打开。

部署脚本：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\deploy_windows_runtime.ps1
```

部署脚本会执行 `windeployqt`，复制 OpenCV DLL，并补齐当前 OpenCV 包里 `libopencv_videoio400.dll` 需要的两个兼容文件名：

```text
libopencv_core400.dll
libopencv_imgcodecs400.dll
```

## Windows 适配点

### 无 PCL 点云导出

Windows 当前构建不依赖 PCL/VTK。点击“生成点云图”时，程序会：

1. 将 `StereoSGBM` 的 16 位视差按 `1/16` 转为浮点视差。
2. 调用 `cv::reprojectImageTo3D()` 生成 `CV_32FC3` 三维坐标矩阵。
3. 过滤无效点、过远点和异常坐标。
4. 将有效点和左图颜色写入 ASCII PLY 文件。

该方案保留了点云生成能力，缺少的是 PCLVisualizer 内置点云窗口。Windows 下可使用外部点云工具查看 `.ply` 文件。

### 控制台 UTF-8

Windows 下在 `main.cpp` 中设置：

```cpp
SetConsoleOutputCP(CP_UTF8);
SetConsoleCP(CP_UTF8);
```

用于减少中文日志乱码。

### 中文路径图片读取

OpenCV 在 Windows 下使用 `cv::imread(path.toStdString())` 容易打不开中文路径。项目改为：

1. 用 Qt `QFile` 读取图片字节。
2. 用 `cv::imdecode()` 解码。

因此可以直接打开：

```text
测试场景/imageL_场景1.png
测试场景/imageR_场景1.png
```

### Windows 摄像头后端

Windows 下使用：

```cpp
capture.open(cameraIndex, cv::CAP_DSHOW)
```

当前相机编号默认为 `0`。如双目相机枚举到其他编号，需要后续改成界面配置项。

### 输出目录

程序启动时自动创建：

```text
imgs/pcd
```

避免保存图片或点云时因目录不存在失败。

## 运行方式

Windows 下直接运行：

```text
build/windows-qt6-mingw/bin/3D_spatial_positioning.exe
```

图片模式测试流程：

1. 点击“打开图片”。
2. 选择 `测试场景/imageL_场景1.png`。
3. 再选择 `测试场景/imageR_场景1.png`。
4. 点击“立体匹配”生成深度图和测距图。
5. 点击测距图进行测距。

相机模式测试流程：

1. 连接双目相机。
2. 点击“打开相机”。
3. 如果无法打开，检查 Windows 设备编号，当前默认是 `0`。

## 当前限制和后续待办

1. Windows 当前构建未启用 PCL/VTK，点云已改为导出 PLY 文件，暂不提供内置 PCLVisualizer 显示窗口。
2. Windows 如需内置点云显示功能，建议准备 ABI 匹配的 MSVC 版本 Qt、OpenCV、PCL、VTK、Boost，再扩展 `.pro` 的 `win32` 分支。
3. 测距点击坐标仍依赖固定偏移，后续应改为基于 `QLabel::geometry()` 和图像缩放比例动态换算。
4. 相机编号当前为常量 `0`，后续应做成 UI 配置。
5. `build/` 目录为构建产物，不应作为源码维护对象。

## 已验证结果

本次整理后已在当前 Windows 环境执行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build_windows_qt6_mingw.ps1
```

结果：

- qmake 成功。
- `mingw32-make -j4` 成功。
- `windeployqt` 部署成功。
- 输出 `build/windows-qt6-mingw/bin/3D_spatial_positioning.exe`。
- GUI 程序启动 5 秒冒烟验证通过，结果为 `started-ok`。

# 版本记录

## v0.1.2-local

- 状态：本地版本，暂不推送 GitHub
- 日期：2026-05-30
- 本地提交：见 `git rev-parse HEAD`
- 本地标签：`v0.1.2-local`
- 上一版本：`v0.1.1-local`

### 本地整理内容

1. Windows 无 PCL/VTK 构建新增 Qt 内置点云预览窗口。
2. 点击“生成点云图”时继续保存 ASCII PLY，同时自动弹出点云预览。
3. 菜单“点云图 -> 打开点云图”支持打开已有 `imgs/pcd/*.ply` 文件预览。
4. 项目文档补充 Windows 点云生成、保存和内置预览说明。

### 验证记录

- Windows 构建命令：`powershell -ExecutionPolicy Bypass -File scripts\build_windows_qt6_mingw.ps1`
- 验证结果：`qmake`、`mingw32-make`、`windeployqt` 通过

## v0.1.1-local

- 状态：本地版本，暂不推送 GitHub
- 日期：2026-05-30
- 本地提交：见 `git rev-parse HEAD`
- 本地标签：`v0.1.1-local`
- 上一版本：`v0.1.0-local`

### 本地整理内容

1. Windows 无 PCL/VTK 构建新增 ASCII PLY 点云导出。
2. 点云导出复用 OpenCV `reprojectImageTo3D()` 三维重投影结果。
3. 将 `StereoSGBM` 的 16 位视差按 `1/16` 转换为浮点视差后再重投影。
4. PLY 顶点包含 `x y z red green blue`，可用 CloudCompare、MeshLab 或 Blender 打开。
5. Ubuntu 或完整 PCL 构建仍保留原 PCL `.pcd` 保存和 PCLVisualizer 显示流程。

### 验证记录

- Windows 构建命令：`powershell -ExecutionPolicy Bypass -File scripts\build_windows_qt6_mingw.ps1`
- 验证结果：qmake、`mingw32-make -j4`、`windeployqt` 通过
- GUI 冒烟验证：`started-ok`

## v0.1.0-local

- 状态：本地版本，暂不推送 GitHub
- 日期：2026-05-30
- 本地提交：见 `git rev-parse HEAD`
- 本地标签：`v0.1.0-local`
- 远端仓库：`https://github.com/shy117/3D_spatial_positioning.git`
- 远端分支：`origin/main`
- 远端基准提交：`62858f2f72c2b31d9e4b32279ed7daa66879b3e7`
- 远端基准时间：`2024-07-20 17:40:33 +0800`
- 远端基准说明：`Add files via upload`

### 本地整理内容

1. 将 Ubuntu 和 Windows 版本整理为一个跨平台 qmake 项目。
2. 保留单一源码目录 `3D_spatial_positioning/`。
3. 删除 `windows_port` 独立项目目录，Windows 构建和部署脚本统一移入 `scripts/`。
4. Windows 下支持 UTF-8 控制台和中文路径测试图片读取。
5. Windows 当前环境默认禁用 PCL/VTK，保留界面、读图、立体匹配、深度图和测距。
6. Windows 无 PCL/VTK 时改为导出 ASCII PLY 点云文件。
7. 新增 Windows 一键构建脚本 `scripts/build_windows_qt6_mingw.ps1`。
8. 新增项目文档 `3D_spatial_positioning_项目文档.md`。

### 验证记录

- Windows 构建命令：`powershell -ExecutionPolicy Bypass -File scripts\build_windows_qt6_mingw.ps1`
- 验证结果：qmake、`mingw32-make -j4`、`windeployqt` 通过
- 输出程序：`build/windows-qt6-mingw/bin/3D_spatial_positioning.exe`
- GUI 冒烟验证：`started-ok`

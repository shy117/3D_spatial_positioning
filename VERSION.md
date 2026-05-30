# 版本记录

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
6. 新增 Windows 一键构建脚本 `scripts/build_windows_qt6_mingw.ps1`。
7. 新增项目文档 `3D_spatial_positioning_项目文档.md`。

### 验证记录

- Windows 构建命令：`powershell -ExecutionPolicy Bypass -File scripts\build_windows_qt6_mingw.ps1`
- 验证结果：qmake、`mingw32-make -j4`、`windeployqt` 通过
- 输出程序：`build/windows-qt6-mingw/bin/3D_spatial_positioning.exe`
- GUI 冒烟验证：`started-ok`

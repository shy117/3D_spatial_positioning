param(
    [string]$ProjectRoot = (Resolve-Path "$PSScriptRoot\..").Path,
    [string]$QtBin = "D:\QT\6.8.0\mingw_64\bin",
    [string]$OpenCvBin = "E:\Workspace\MyGit\videoCapture\videoCapture\3rdparty\bin"
)

$ErrorActionPreference = "Stop"

$binDir = Join-Path $ProjectRoot "build\windows-qt6-mingw\bin"
$exe = Join-Path $binDir "3D_spatial_positioning.exe"

if (!(Test-Path $exe)) {
    throw "Executable not found: $exe"
}

& (Join-Path $QtBin "windeployqt.exe") $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Copy-Item -Force (Join-Path $OpenCvBin "*.dll") $binDir

# The available OpenCV package mixes videoio400 with mostly 401 DLLs. videoio400
# declares these 400 filenames, so provide aliases from the matching local DLLs.
Copy-Item -Force (Join-Path $binDir "libopencv_core401.dll") (Join-Path $binDir "libopencv_core400.dll")
Copy-Item -Force (Join-Path $binDir "libopencv_imgcodecs401.dll") (Join-Path $binDir "libopencv_imgcodecs400.dll")

Write-Host "Runtime deployed to $binDir"

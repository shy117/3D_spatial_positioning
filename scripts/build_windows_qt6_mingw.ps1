param(
    [string]$ProjectRoot = (Resolve-Path "$PSScriptRoot\..").Path,
    [string]$QtBin = "D:\QT\6.8.0\mingw_64\bin",
    [string]$MingwBin = "D:\QT\Tools\mingw1310_64\bin"
)

$ErrorActionPreference = "Stop"

$buildDir = Join-Path $ProjectRoot "build\windows-qt6-mingw"
$projectFile = Join-Path $ProjectRoot "3D_spatial_positioning\3D_spatial_positioning.pro"

New-Item -ItemType Directory -Force $buildDir | Out-Null

$env:PATH = "$MingwBin;$QtBin;$env:PATH"
& (Join-Path $QtBin "qmake.exe") $projectFile -o (Join-Path $buildDir "Makefile")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $MingwBin "mingw32-make.exe") -C $buildDir -j4
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& powershell -ExecutionPolicy Bypass -File (Join-Path $ProjectRoot "scripts\deploy_windows_runtime.ps1")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "Build finished: $(Join-Path $buildDir 'bin\3D_spatial_positioning.exe')"

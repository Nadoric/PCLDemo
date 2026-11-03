$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath("$PSScriptRoot\..")
$cmakeExe = 'D:\Program Files\CMake\bin\cmake.exe'
$msbuildExe = 'D:\Program Files\Microsoft Visual Studio\18\Enterprise\MSBuild\Current\Bin\MSBuild.exe'
$qtRoot = 'D:\software\Qt\6.12.0\msvc2022_64'
$pclRoot = 'D:\PCL'
$opencvRoot = 'D:\opencv\5.0.0\opencv'
# 仅修改当前脚本进程及其子进程的 PATH，不改系统环境变量。
$env:PATH = "$pclRoot\bin;$pclRoot\3rdParty\VTK\bin;$pclRoot\3rdParty\FLANN\bin;$pclRoot\3rdParty\Qhull\bin;C:\Program Files\OpenNI2\Redist;$qtRoot\bin;$env:PATH"
$env:QT_QPA_PLATFORM_PLUGIN_PATH = "$qtRoot\plugins\platforms"
$env:PATH = "$opencvRoot\build\x64\vc16\bin;$env:PATH"

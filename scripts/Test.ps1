param([ValidateSet('Debug','Release')][string]$Configuration = 'Release')
. "$PSScriptRoot\Environment.ps1"
$application = "$projectRoot\bin\$Configuration\PCLDemo.exe"
if (-not [System.IO.File]::Exists($application)) { throw 'Build the selected configuration first.' }
$report = "$projectRoot\artifacts\self-test-$Configuration.txt"
# 通过控制台启动器等待 GUI 子系统程序，兼容 Windows PowerShell 5 和 PowerShell 7。
# 这里只使用 CMake 的进程启动功能，不执行 CMake 配置或编译。
& $cmakeExe -E env -- $application --self-test --report $report
$testExitCode = $LASTEXITCODE
if ($testExitCode -ne 0) {
    if ([System.IO.File]::Exists($report)) { [System.IO.File]::ReadAllText($report) }
    throw "Self-test failed. Exit code: $testExitCode"
}
[System.IO.File]::ReadAllText($report)

param([ValidateSet('Debug','Release')][string]$Configuration = 'Release')
. "$PSScriptRoot\Environment.ps1"
$application = Join-Path $projectRoot "bin\$Configuration\PCLDemo.exe"
if (-not (Test-Path -LiteralPath $application)) { & "$PSScriptRoot\Build.ps1" -Configuration $Configuration }
Start-Process -FilePath $application -WorkingDirectory $projectRoot

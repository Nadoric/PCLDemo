param([ValidateSet('Debug','Release')][string]$Configuration = 'Release')
. "$PSScriptRoot\Environment.ps1"
Push-Location -LiteralPath $projectRoot
try {
    & $msbuildExe "$projectRoot\PCLDemo.sln" /m:2 /p:Configuration=$Configuration /p:Platform=x64 /v:minimal /nologo
    if ($LASTEXITCODE -ne 0) { throw 'Build failed.' }
    Write-Host "Build OK: $projectRoot\bin\$Configuration\PCLDemo.exe"
} finally { Pop-Location }

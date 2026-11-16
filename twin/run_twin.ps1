# Builds (if needed) and starts the RIP digital twin in a window. Usage:  powershell -File twin\run_twin.ps1
$ue = "D:\Unrealengine 5\UE_5.3\Engine"
$proj = Join-Path $PSScriptRoot "RIPTwin\RIPTwin.uproject"
& "$ue\Build\BatchFiles\Build.bat" RIPTwinEditor Win64 Development "-Project=$proj" -WaitMutex -NoHotReload -MaxParallelActions=2
if ($LASTEXITCODE -ne 0) { Write-Error "build failed (low memory? close SolidWorks)"; exit 1 }
Start-Process "$ue\Binaries\Win64\UnrealEditor.exe" -ArgumentList "`"$proj`"", '-game', '-windowed', '-ResX=1600', '-ResY=900'

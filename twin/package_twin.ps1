# Packages the RIP digital twin as a standalone Windows app (Shipping): no Unreal editor, Visual Studio or build
# cache needed to run it. Output: dist\RIPTwin\Windows\RIPTwin.exe (dist\ is not committed).
# Usage:  powershell -File twin\package_twin.ps1          (takes 10-30 min the first time: compile + cook)
# Run the result:  dist\RIPTwin\Windows\RIPTwin.exe -windowed -ResX=1600 -ResY=900
# It listens for RIP1 UDP packets on port 7400 (Windows may ask once to allow it on private networks).
$ue = "D:\Unrealengine 5\UE_5.3\Engine"
$proj = Join-Path $PSScriptRoot "RIPTwin\RIPTwin.uproject"
$out = Join-Path (Split-Path $PSScriptRoot -Parent) "dist\RIPTwin"
& "$ue\Build\BatchFiles\RunUAT.bat" BuildCookRun "-project=$proj" -noP4 -platform=Win64 -clientconfig=Shipping `
    -build -cook -stage -pak -archive "-archivedirectory=$out" -nodebuginfo -utf8output `
    "-UbtArgs=-MaxParallelActions=2"
if ($LASTEXITCODE -ne 0) { Write-Error "packaging failed (see the log above; low memory? close SolidWorks)"; exit 1 }
Write-Host "Packaged: $out\Windows\RIPTwin.exe"

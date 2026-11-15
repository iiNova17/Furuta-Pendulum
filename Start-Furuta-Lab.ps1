$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$taskPython = Join-Path $taskRoot '.venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $taskPython)) {
    throw 'Run Setup-Furuta-Lab.ps1 once to install the local Python environment.'
}
& $taskPython (Join-Path $taskRoot 'desktop/main.py')

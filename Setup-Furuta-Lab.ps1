param([string]$Python = 'python')
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$taskEnvironment = Join-Path $taskRoot '.venv'
if (-not (Test-Path -LiteralPath (Join-Path $taskEnvironment 'Scripts/python.exe'))) {
    & $Python -m venv $taskEnvironment
    if ($LASTEXITCODE -ne 0) { throw 'Python 3.12+ is required to create the local environment.' }
}
& (Join-Path $taskEnvironment 'Scripts/python.exe') -m pip install -r (Join-Path $taskRoot 'desktop/requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }

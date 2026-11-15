param(
    [ValidateSet('build', 'upload', 'monitor', 'ports')]
    [string]$Action = 'build',
    [string]$Port
)
$ErrorActionPreference = 'Stop'
$taskPio = Join-Path $env:USERPROFILE '.platformio/penv/Scripts/platformio.exe'
if (-not (Test-Path -LiteralPath $taskPio)) {
    $taskPioCommand = Get-Command platformio -ErrorAction SilentlyContinue
    if (-not $taskPioCommand) { throw 'Install PlatformIO IDE or PlatformIO Core first.' }
    $taskPio = $taskPioCommand.Source
}
$taskTemp = Join-Path $PSScriptRoot '.pio/tmp'
New-Item -ItemType Directory -Path $taskTemp -Force | Out-Null
$taskPreviousTemp = $env:TEMP
$taskPreviousTmp = $env:TMP
$taskExit = 1
Push-Location -LiteralPath $PSScriptRoot
try {
    $env:TEMP = [IO.Path]::GetFullPath($taskTemp)
    $env:TMP = $env:TEMP
    switch ($Action) {
        'build'   { & $taskPio run }
        'ports'   { & $taskPio device list }
        'upload'  {
            if ($Port) { & $taskPio run --target upload --upload-port $Port }
            else { & $taskPio run --target upload }
        }
        'monitor' {
            if ($Port) { & $taskPio device monitor --port $Port }
            else { & $taskPio device monitor }
        }
    }
    $taskExit = $LASTEXITCODE
} finally {
    $env:TEMP = $taskPreviousTemp
    $env:TMP = $taskPreviousTmp
    Pop-Location
}
exit $taskExit

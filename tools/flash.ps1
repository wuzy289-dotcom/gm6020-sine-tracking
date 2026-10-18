# 一键编译 + 烧录
#   powershell -File tools/flash.ps1 -Interface stlink
#   powershell -File tools/flash.ps1 -Interface jlink -BuildOnly
param(
    [ValidateSet('stlink', 'jlink', 'cmsis-dap')]
    [string]$Interface = 'jlink',
    [switch]$BuildOnly
)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$elf  = Join-Path $root 'build/gm6020_sine.elf'

Push-Location $root
try {
    Write-Host '== build ==' -ForegroundColor Cyan
    cmake --build --preset Debug
    if ($LASTEXITCODE -ne 0) { throw 'build failed' }

    if ($BuildOnly) { return }

    $openocd = (Get-Command openocd -ErrorAction SilentlyContinue).Source
    if (-not $openocd) {
        foreach ($p in @(
            'D:\tools\openocd-1f3f583-i686-w64-mingw32\bin\openocd.exe',
            'C:\Program Files\OpenOCD\bin\openocd.exe',
            'C:\xpack-openocd\bin\openocd.exe')) {
            if (Test-Path $p) { $openocd = $p; break }
        }
    }
    if (-not $openocd) {
        throw 'openocd 没找到, 请安装 OpenOCD 或把它加到 PATH'
    }

    Write-Host "== flash via $Interface ==" -ForegroundColor Cyan
    & $openocd -f "interface/$Interface.cfg" -f target/stm32f4x.cfg `
        -c "program `"$elf`" verify reset exit"
    if ($LASTEXITCODE -ne 0) { throw 'flash failed' }
    Write-Host 'done.' -ForegroundColor Green
}
finally {
    Pop-Location
}

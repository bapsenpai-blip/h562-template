# Chạy một lần từ thư mục gốc project:  powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
$ErrorActionPreference = "Stop"
Set-Location (Split-Path $PSScriptRoot -Parent)

if (-not (Test-Path ".git")) { git init }

$subs = @{
    "drivers/cmsis_core"           = "https://github.com/STMicroelectronics/cmsis_core.git"
    "drivers/cmsis-device-h5"      = "https://github.com/STMicroelectronics/cmsis-device-h5.git"
    "drivers/stm32h5xx-hal-driver" = "https://github.com/STMicroelectronics/stm32h5xx-hal-driver.git"
}

foreach ($path in $subs.Keys) {
    if (-not (Test-Path $path)) {
        Write-Host ">> Thêm submodule $path"
        git submodule add $subs[$path] $path
    }
}
git submodule update --init --recursive

# Copy linker script vào linker/ (file này bạn sẽ hay chỉnh nên để trong project)
New-Item -ItemType Directory -Force -Path "linker" | Out-Null
if (-not (Test-Path "linker/STM32H562xx_FLASH.ld")) {
    $ld = Get-ChildItem -Recurse -Path "drivers/cmsis-device-h5" -Filter "STM32H562xx_FLASH.ld" | Select-Object -First 1
    if ($null -eq $ld) {
        Write-Warning "Không tìm thấy STM32H562xx_FLASH.ld — lấy từ project CubeMX hoặc repo STM32CubeH5 rồi đặt vào linker/"
    } else {
        Copy-Item $ld.FullName "linker/"
        Write-Host ">> Đã copy $($ld.FullName) -> linker/"
    }
}

Write-Host "`nXong. Mở thư mục này bằng VS Code, chọn preset 'Debug' rồi Build."


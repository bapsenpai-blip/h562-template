#!/usr/bin/env bash
# Chạy một lần từ thư mục gốc project:  bash scripts/setup.sh
set -euo pipefail
cd "$(dirname "$0")/.."

[ -d .git ] || git init

add_sub() {
    [ -d "$1" ] || { echo ">> Thêm submodule $1"; git submodule add "$2" "$1"; }
}
add_sub drivers/cmsis_core           https://github.com/STMicroelectronics/cmsis_core.git
add_sub drivers/cmsis-device-h5      https://github.com/STMicroelectronics/cmsis-device-h5.git
add_sub drivers/stm32h5xx-hal-driver https://github.com/STMicroelectronics/stm32h5xx-hal-driver.git
git submodule update --init --recursive

mkdir -p linker
if [ ! -f linker/STM32H562xx_FLASH.ld ]; then
    ld=$(find drivers/cmsis-device-h5 -name 'STM32H562xx_FLASH.ld' | head -n1 || true)
    if [ -z "$ld" ]; then
        echo "!! Không tìm thấy STM32H562xx_FLASH.ld — lấy từ CubeMX/STM32CubeH5 rồi đặt vào linker/"
    else
        cp "$ld" linker/ && echo ">> Đã copy $ld -> linker/"
    fi
fi

echo; echo "Xong. Mở thư mục này bằng VS Code, chọn preset 'Debug' rồi Build."

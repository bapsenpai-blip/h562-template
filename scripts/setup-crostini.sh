#!/usr/bin/env bash
# Cài môi trường build/debug cho project trên Linux (ChromeOS Crostini / Debian / Ubuntu).
# Chạy từ thư mục gốc project:   bash scripts/setup-crostini.sh
set -euo pipefail
cd "$(dirname "$0")/.."

ARCH=$(uname -m)
echo ">> Kiến trúc máy: $ARCH"

echo ">> Cài công cụ build (cần sudo)"
sudo apt-get update
sudo apt-get install -y git cmake ninja-build clangd \
    gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi \
    gdb-multiarch usbutils

# cortex-debug tìm arm-none-eabi-gdb; Debian chỉ có gdb-multiarch -> tạo link
if ! command -v arm-none-eabi-gdb >/dev/null; then
    mkdir -p "$HOME/.local/bin"
    ln -sf "$(command -v gdb-multiarch)" "$HOME/.local/bin/arm-none-eabi-gdb"
    echo ">> Đã tạo ~/.local/bin/arm-none-eabi-gdb -> gdb-multiarch"
fi

echo ">> Lấy submodule HAL/CMSIS"
git submodule update --init --recursive

echo
arm-none-eabi-gcc --version | head -1
cmake --version | head -1
ninja --version

echo
if [ -z "${STM32CLT_PATH:-}" ]; then
    CLT=$(ls -d /opt/st/stm32cubeclt_* 2>/dev/null | sort -V | tail -1 || true)
    if [ -n "$CLT" ]; then
        echo ">> Thấy STM32CubeCLT ở $CLT nhưng STM32CLT_PATH chưa đặt. Thêm vào ~/.profile:"
        echo "     export STM32CLT_PATH=$CLT"
        echo "     export PATH=\"\$STM32CLT_PATH/STM32CubeProgrammer/bin:\$PATH\""
    else
        echo "!! Chưa có STM32CubeCLT (cần để nạp/debug qua ST-Link)."
        if [ "$ARCH" = "x86_64" ]; then
            echo "   Tải bản Linux tại https://www.st.com/stm32cubeclt rồi cài, sau đó chạy lại script này."
        else
            echo "   Máy $ARCH: STM32CubeCLT chưa có bản Linux cho kiến trúc này."
            echo "   Build vẫn chạy bình thường; nạp/debug xem README phần Crostini."
        fi
    fi
fi

echo
echo "Xong. Mở VS Code tại thư mục này, chọn preset 'Debug' và Build (Ctrl+Shift+B)."

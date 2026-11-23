#!/usr/bin/env bash
# Build OpenOCD fork của ST (có hỗ trợ STM32H5) + tải SVD, cho Linux/Crostini (aarch64 không có CubeCLT).
# Upstream OpenOCD chưa hỗ trợ flash STM32H5 nên phải dùng bản của ST.
set -euo pipefail
sudo apt-get install -y libusb-1.0-0-dev libhidapi-dev libftdi1-dev libjaylink-dev \
    autoconf automake libtool pkg-config texinfo gdb-multiarch git curl
mkdir -p ~/src ~/.local/bin ~/.local/share/svd
[ -d ~/src/openocd-st ] || git clone --depth 1 -b openocd-cubeide-r7 https://github.com/STMicroelectronics/OpenOCD.git ~/src/openocd-st
cd ~/src/openocd-st
git submodule update --init --depth 1 jimtcl
./bootstrap && ./configure --prefix="$HOME/.local/openocd-st" --enable-stlink --disable-werror
make -j"$(nproc)" && make install
ln -sf "$(command -v gdb-multiarch)" ~/.local/bin/arm-none-eabi-gdb
curl -fL -o ~/.local/share/svd/STM32H562.svd https://raw.githubusercontent.com/modm-io/cmsis-svd-stm32/main/stm32h5/STM32H562.svd
echo "Xong. VS Code: chọn 'Debug (OpenOCD, Linux/Crostini)'. Flash: task 'Flash (ST-Link SWD)'."

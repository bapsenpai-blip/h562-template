# Template STM32H562VGT6 (WeAct) — CMake + HAL + CMSIS

Project tối giản cho STM32H562: CMSIS và HAL của ST được thêm dưới dạng git submodule, phần còn lại (`src/`, `include/`) là code của bạn. Build bằng CMake + Ninja, nạp và debug qua ST-Link hoặc USB DFU.

```
h562-template/
├── CMakeLists.txt          # cấu hình build chính
├── CMakePresets.json       # preset Debug / Release
├── cmake/                  # toolchain arm-none-eabi-gcc
├── drivers/                # (tạo bởi setup) CMSIS + HAL của ST — KHÔNG sửa
├── linker/                 # STM32H562xx_FLASH.ld (copy bởi setup, được phép sửa)
├── include/                # header của bạn + stm32h5xx_hal_conf.h
├── src/                    # code của bạn (main.c, system_clock.c, stm32h5xx_it.c, ...)
├── scripts/setup.ps1|.sh   # tải driver, copy linker script
└── .vscode/                # build/flash task + cấu hình debug
```

---

## Bước 1 — Cài công cụ (một lần)

> **Cài CubeCLT ở ổ khác (vd. D:)?** Không sao. Sau khi cài, chạy một lần:
> ```powershell
> powershell -ExecutionPolicy Bypass -File scripts\setup-clt.ps1
> ```
> Script tự tìm CubeCLT, đặt biến `STM32CLT_PATH` và thêm các thư mục `bin` vào PATH của user. Đóng hẳn VS Code rồi mở lại. Khi nâng cấp CubeCLT, chạy lại script này là xong.

| Công cụ | Dùng để | Ghi chú |
|---|---|---|
| **STM32CubeCLT** (st.com) | GCC ARM, CMake, Ninja, STM32_Programmer_CLI, ST-LINK_gdbserver, file SVD | Gói tất cả trong một. Cài vào `C:\ST\STM32CubeCLT_x.y.z` |
| **Git** | Lấy driver dạng submodule | |
| **STM32CubeMX** (tùy chọn) | Tính clock, tra pin | Không bắt buộc, nhưng rất tiện để kiểm tra cấu hình |
| Extension VS Code | C/C++, CMake Tools, Cortex-Debug, STM32Cube for VS Code | Có trong `.vscode/extensions.json` |

(Cách thủ công, nếu không dùng script) **thêm các thư mục sau vào PATH** (đổi ổ đĩa / phiên bản cho đúng):

```
C:\ST\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin
C:\ST\STM32CubeCLT_1.18.0\CMake\bin
C:\ST\STM32CubeCLT_1.18.0\Ninja\bin
C:\ST\STM32CubeCLT_1.18.0\STM32CubeProgrammer\bin
```

Kiểm tra trong terminal mới:

```
arm-none-eabi-gcc --version
cmake --version
ninja --version
STM32_Programmer_CLI --version
```

---

## Bước 2 — Lấy driver

Giải nén template, mở terminal tại thư mục gốc và chạy:

```powershell
# Windows
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
```
```bash
# Linux / macOS
bash scripts/setup.sh
```

Script sẽ:
1. `git init` (nếu chưa có).
2. Thêm 3 submodule chính thức của ST vào `drivers/`:
   - `cmsis_core` — CMSIS Core (core_cm33.h, ...)
   - `cmsis-device-h5` — header thanh ghi `stm32h562xx.h`, `system_stm32h5xx.c`, startup `.s`, linker script
   - `stm32h5xx-hal-driver` — HAL + LL
3. Copy `STM32H562xx_FLASH.ld` vào `linker/`.

> Nếu script báo không tìm thấy linker script: tạo một project bất kỳ cho `STM32H562VGTx` trong CubeMX (Toolchain = CMake), lấy file `.ld` ra và đặt vào `linker/STM32H562xx_FLASH.ld`.

**Khóa phiên bản driver** (khuyến nghị cho dự án dài hạn):

```
cd drivers/stm32h5xx-hal-driver
git tag                 # xem các bản phát hành
git checkout v1.x.y     # chọn một tag
cd ../..
git add drivers && git commit -m "Pin HAL v1.x.y"
```

Làm tương tự với `cmsis-device-h5`. Khi muốn nâng cấp, chỉ cần checkout tag mới.

---

## Bước 3 — Chỉnh cho đúng board

1. Đã chạy `scripts/setup-clt.ps1` (Bước 1) — các file `.vscode/` tự dùng `STM32CLT_PATH`, không cần sửa đường dẫn.
2. **`include/board.h`** — đổi port/pin LED theo schematic của WeAct.
3. **`include/stm32h5xx_hal_conf.h`** — file này được **tạo tự động lần đầu chạy CMake** từ template của ST. Sau đó mở ra:
   - Đặt `HSE_VALUE` đúng tần số thạch anh trên board (xem schematic).
   - Comment các `#define HAL_xxx_MODULE_ENABLED` không dùng để build nhanh và nhẹ hơn. Giữ lại tối thiểu: `HAL_MODULE_ENABLED`, `CORTEX`, `RCC`, `GPIO`, `PWR`, `FLASH`, `DMA`, `EXTI`.
4. **`src/system_clock.c`** — mặc định chạy 250 MHz từ HSI (không cần thạch anh). Khi cần HSE (USB, CAN-FD, UART tốc độ cao chính xác), mở CubeMX → tab Clock Configuration để lấy hệ số PLL rồi sửa lại.

---

## Bước 4 — Build

Mở thư mục bằng VS Code. CMake Tools sẽ hỏi chọn preset → chọn **Debug**.

- **Build:** `Ctrl+Shift+B` (task *Build (Debug)*), hoặc nút Build trên thanh trạng thái.
- Dòng lệnh tương đương:

```
cmake --preset Debug
cmake --build --preset Debug
```

Kết quả nằm trong `build/Debug/`: `h562_app.elf`, `.hex`, `.bin`, `.map`. Cuối log có bảng dung lượng FLASH/RAM đã dùng.

---

## Bước 5 — Nạp chương trình

**Qua ST-Link (SWD)** — nối SWDIO, SWCLK, GND, 3V3 (và NRST nếu có):
- `Ctrl+Shift+P` → *Tasks: Run Task* → **Flash (ST-Link SWD)**

**Qua USB-C (DFU, không cần ST-Link):**
1. Giữ nút **BOOT0**, nhấn **RESET** (hoặc cắm cáp), thả BOOT0.
2. Run Task → **Flash (USB DFU)**.
3. Nhấn RESET để chạy chương trình.

> Windows có thể cần driver DFU: cài STM32CubeProgrammer bản đầy đủ hoặc dùng driver đi kèm CubeCLT.

---

## Bước 6 — Debug

1. Cắm ST-Link.
2. Tab **Run and Debug** → chọn **Debug (ST-Link)** → `F5`.
3. Chương trình dừng tại `main()`. Có breakpoint, xem biến, và tab **XPERIPHERALS** để xem thanh ghi ngoại vi (nhờ file SVD).

Nếu báo không tìm thấy gdb server: mở terminal mới, chạy `echo $env:STM32CLT_PATH` — nếu rỗng thì chạy lại `scripts/setup-clt.ps1` và khởi động lại VS Code.

> Bạn cũng có thể dùng extension **STM32Cube for VS Code** của ST: nó nhận diện project CMake và tự tạo cấu hình debug riêng. Hai cách dùng song song được.

---

## Phát triển tiếp

- **Thêm file:** đặt `.c` vào `src/` (kể cả thư mục con), `.h` vào `include/`. CMake tự quét, không phải sửa `CMakeLists.txt`.
- **Thêm ngoại vi:** bật module trong `stm32h5xx_hal_conf.h`, viết hàm `HAL_xxx_MspInit()` (clock, GPIO, NVIC) trong code của bạn, thêm `xxx_IRQHandler` vào `stm32h5xx_it.c` nếu dùng ngắt.
- **Dùng LL driver:** bỏ comment `USE_FULL_LL_DRIVER` trong `CMakeLists.txt`.
- **Release:** chọn preset **Release** (tối ưu `-O2`, không debug info).
- **Thư viện riêng:** tạo thư mục `lib/tên_lib/` có `CMakeLists.txt` riêng, rồi `add_subdirectory()` + `target_link_libraries()`.
- **Đổi sang chip STM32 khác:** sửa `MCU_DEVICE`, `MCU_STARTUP`, `MCU_LDSCRIPT`, `MCU_FLAGS` và submodule driver tương ứng.

## Lỗi thường gặp

| Lỗi | Cách sửa |
|---|---|
| `Không tìm thấy drivers/...` | Chưa chạy `scripts/setup`. |
| `arm-none-eabi-gcc not found` | Chưa thêm vào PATH, hoặc đặt `TOOLCHAIN_PREFIX` trong preset. |
| `unrecognized option '--no-warn-rwx-segments'` | Toolchain cũ — xóa dòng đó trong `CMakeLists.txt`. |
| `undefined reference to HAL_xxx` | Module chưa bật trong `stm32h5xx_hal_conf.h`. |
| LED không nháy | Sai pin trong `board.h`, hoặc LED active-low. |
| Kẹt trong `HardFault_Handler` | Đặt breakpoint tại đó, xem thanh ghi CFSR trong XPERIPHERALS → SCB. |
| Nạp DFU không thấy thiết bị | Chưa vào bootloader (BOOT0) hoặc thiếu driver USB. |

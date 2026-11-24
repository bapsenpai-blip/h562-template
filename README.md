# Autopilot UAV Cánh Bằng — STM32H562VGT6

Firmware điều khiển bay (Autopilot) cho UAV cánh bằng chạy trên vi điều khiển **WeAct STM32H562VGT6** (ARM Cortex-M33 @ 250 MHz, HSE 8 MHz), xây dựng trên nền tảng **STM32 HAL + CMSIS** với **CMake + Ninja**.

---

## 1. Kiến trúc & tính năng chính

- **Thu thập cảm biến bất đồng bộ (GPDMA1 + FIFO):**
  - **IMU (ICM-45686 / GY-601N01):** đọc gia tốc và con quay ở `1600 Hz` qua `SPI1` sử dụng `GPDMA1` (`CH2` RX / `CH3` TX) kết hợp FIFO phần cứng (chế độ `STREAM`, watermark 4 frame, tính `dt` chính xác từ timestamp FIFO).
  - **Áp suất kế (BMP388):** đọc áp suất và nhiệt độ qua `I2C1` sử dụng `GPDMA1` (`CH0` RX), hỗ trợ FIFO dữ liệu đã lọc IIR để tính độ cao tương đối.
  - **GPS (u-blox UBX NAV-PVT)** & **La bàn (QMC5883L):** đã có sẵn driver trong `src/` (`ubx_gps.c`, `qmc5883l.c`), bật/tắt qua macro trong [include/config.h](include/config.h) (đề xuất dùng module **MicoAir MGA01**).
- **Xử lý tín hiệu & ước lượng tư thế:**
  - Lọc thông thấp Biquad (`biquadfilter.c`) cho tín hiệu gyro và accel.
  - Bộ lọc bổ sung **Mahony** (`mahony.c`, `quaternion.c`) ước lượng góc tư thế (`roll`, `pitch`, `yaw`).
- **Telemetry MAVLink v2 qua ESP32 (DroneBridge):**
  - Đóng gói bản tin **MAVLink v2** (`mavlink_send.c`) vào **TX FIFO (ring buffer 1024 byte)**.
  - Truyền không chặn qua `USART1` (115200 bps) bằng `GPDMA1` (`CH4` TX / `CH5` RX) tới module **ESP32 chạy DroneBridge** về trạm mặt đất (Mission Planner / QGroundControl).
- **Thiết kế phần cứng:** Schematic bo mạch carrier đã hoàn thiện ([MachCanhBang.kicad_sch](MachCanhBang.kicad_sch)), layout PCB đang được phát triển song song với firmware.

---

## 2. Sơ đồ kết nối & phân bổ GPDMA1

| Khối | Linh kiện | Giao tiếp | Chân STM32H562 | Kênh GPDMA1 |
|---|---|---|---|---|
| IMU | ICM-45686 | SPI1 | `PA5/PA6/PA7` (+ CS/INT) | `CH2` (RX), `CH3` (TX) |
| Baro | BMP388 | I2C1 | `PB6` (SCL), `PB7` (SDA) | `CH0` (RX) |
| Mag *(tùy chọn)* | QMC5883L | I2C1 | `PB6` (SCL), `PB7` (SDA) | `CH0` (RX) |
| GPS *(tùy chọn)* | u-blox UBX (`MicoAir MGA01`) | USART3 | `PB10` (TX), `PB11` (RX) | `CH1` (RX) |
| Telemetry | ESP32 (DroneBridge) | USART1 | `PA9` (TX), `PA10` (RX) | `CH4` (TX), `CH5` (RX) |
| Timer hệ thống | Free-running 1 µs | TIM5 | Nội bộ (`250 MHz / 250`) | — |

Bật/tắt các cảm biến tùy chọn trong [include/config.h](include/config.h):
```c
//#define USE_GPS   /* u-blox UBX qua USART3 */
//#define USE_MAG   /* QMC5883L qua I2C1 */
#define USE_BARO    /* BMP388 qua I2C1 */
```

---

## 3. Cấu trúc thư mục

```text
h562-template/
├── CMakeLists.txt          # Cấu hình build chính (HAL + App)
├── CMakePresets.json       # Preset Debug / Release (Ninja + arm-none-eabi-gcc)
├── MachCanhBang.kicad_sch  # Schematic bo mạch carrier autopilot
├── cmake/                  # Toolchain file cho arm-none-eabi-gcc
├── drivers/                # Submodule CMSIS + STM32H5 HAL (không chỉnh sửa)
├── linker/                 # STM32H562xx_FLASH.ld
├── include/                # Header ứng dụng, config.h, imu/, mavlink/
├── src/                    # Mã nguồn chính (main.c, board.c, msp.c, driver...)
└── scripts/                # Script cài đặt CubeCLT PATH & tải driver submodule
```

---

## 4. Cài đặt & Build nhanh (CMake)

### Chuẩn bị môi trường (chỉ làm 1 lần)
1. Cài **STM32CubeCLT** (bao gồm `arm-none-eabi-gcc`, `CMake`, `Ninja`, `STM32_Programmer_CLI`, `ST-LINK_gdbserver`) và **Git**.
   - Nếu cài CubeCLT trên Windows ở ổ khác, chạy `powershell -ExecutionPolicy Bypass -File scripts\setup-clt.ps1` để tự cấu hình PATH.
2. Tải driver CMSIS/HAL và linker script (nếu chưa có thư mục `drivers/`):
   ```bash
   # Linux / macOS
   bash scripts/setup.sh
   ```
   ```powershell
   # Windows
   powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
   ```

### Build, Nạp & Debug
- **Build bằng CLI:**
  ```bash
  cmake --preset Debug          # hoặc Release (-O2)
  cmake --build --preset Debug
  ```
  File đầu ra nằm tại `build/Debug/`: `h562_app.elf`, `h562_app.hex`, `h562_app.bin`, `h562_app.map`.
- **Build/Flash/Debug trong VS Code:**
  - Build: `Ctrl+Shift+B` (*Build (Debug)*).
  - Nạp ST-Link (SWD): `Ctrl+Shift+P` → *Tasks: Run Task* → **Flash (ST-Link SWD)**.
  - Nạp USB DFU: giữ **BOOT0** + nhấn **RESET**, sau đó chạy task **Flash (USB DFU)**.
  - Debug: tab **Run and Debug** → **Debug (ST-Link)** (`F5`).

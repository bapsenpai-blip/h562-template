cmake_minimum_required(VERSION 3.22)

project(h562_app C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # cho IntelliSense / clangd

# ============================================================================
#  Cấu hình chip
# ============================================================================
set(MCU_DEVICE  STM32H562xx)
set(MCU_STARTUP startup_stm32h562xx.s)
set(MCU_LDSCRIPT ${CMAKE_SOURCE_DIR}/linker/STM32H562xx_FLASH.ld)

set(MCU_FLAGS
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
)

# ============================================================================
#  Đường dẫn driver (git submodule trong thư mục drivers/)
# ============================================================================
set(DRV_DIR    ${CMAKE_SOURCE_DIR}/drivers)
set(CMSIS_CORE ${DRV_DIR}/cmsis_core)
set(CMSIS_DEV  ${DRV_DIR}/cmsis-device-h5)
set(HAL_DIR    ${DRV_DIR}/stm32h5xx-hal-driver)

foreach(_d ${CMSIS_CORE} ${CMSIS_DEV} ${HAL_DIR})
    if(NOT EXISTS ${_d})
        message(FATAL_ERROR
            "Không tìm thấy ${_d}\n"
            "Hãy chạy scripts/setup.ps1 (Windows) hoặc scripts/setup.sh (Linux/macOS) trước.")
    endif()
endforeach()

# Tự dò thư mục chứa core_cm33.h (repo cmsis_core có thể để ở Include/ hoặc Core/Include/)
file(GLOB_RECURSE _CORE_CM33_H ${CMSIS_CORE}/core_cm33.h)
if(NOT _CORE_CM33_H)
    message(FATAL_ERROR
        "Không tìm thấy core_cm33.h trong ${CMSIS_CORE}\n"
        "Submodule có thể bị trống — chạy: git submodule update --init --recursive")
endif()
list(SORT _CORE_CM33_H)
list(GET _CORE_CM33_H 0 _CORE_CM33_H)
get_filename_component(CMSIS_CORE_INC ${_CORE_CM33_H} DIRECTORY)
message(STATUS "CMSIS Core include: ${CMSIS_CORE_INC}")

if(NOT EXISTS ${MCU_LDSCRIPT})
    message(FATAL_ERROR "Thiếu linker script ${MCU_LDSCRIPT} — chạy lại script setup.")
endif()

# Tự tạo stm32h5xx_hal_conf.h từ template nếu chưa có (chỉ lần đầu)
set(HAL_CONF ${CMAKE_SOURCE_DIR}/include/stm32h5xx_hal_conf.h)
if(NOT EXISTS ${HAL_CONF})
    file(COPY_FILE ${HAL_DIR}/Inc/stm32h5xx_hal_conf_template.h ${HAL_CONF})
    message(WARNING
        "Đã tạo include/stm32h5xx_hal_conf.h từ template.\n"
        "→ Mở file, chỉnh HSE_VALUE cho đúng thạch anh của board và tắt các module không dùng.")
endif()

# ============================================================================
#  Thư viện tĩnh: CMSIS + HAL (code của ST, không sửa)
# ============================================================================
file(GLOB HAL_SOURCES ${HAL_DIR}/Src/*.c)
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*_template\\.c$")

add_library(stm32h5_hal STATIC
    ${HAL_SOURCES}
    ${CMSIS_DEV}/Source/Templates/system_stm32h5xx.c
)

target_include_directories(stm32h5_hal PUBLIC
    ${CMAKE_SOURCE_DIR}/include          # chứa stm32h5xx_hal_conf.h
    ${CMSIS_CORE_INC}
    ${CMSIS_DEV}/Include
    ${HAL_DIR}/Inc
)

target_compile_definitions(stm32h5_hal PUBLIC
    ${MCU_DEVICE}
    USE_HAL_DRIVER
    HSE_VALUE=8000000UL
    # USE_FULL_LL_DRIVER                 # bật nếu dùng LL driver
)

target_compile_options(stm32h5_hal PUBLIC
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
)
# Không cần cảnh báo từ code của ST
target_compile_options(stm32h5_hal PRIVATE -w)

# ============================================================================
#  Ứng dụng của bạn
# ============================================================================
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/*.c)

list(APPEND APP_SOURCES
    ${CMAKE_SOURCE_DIR}/include/imu/inv_imu_driver.c
    ${CMAKE_SOURCE_DIR}/include/imu/inv_imu_transport.c)

add_executable(${PROJECT_NAME}
    ${APP_SOURCES}
    ${CMSIS_DEV}/Source/Templates/gcc/${MCU_STARTUP}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/mavlink)   # cho phép cả <mavlink/common/mavlink.h> lẫn <common/mavlink.h>

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wshadow
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
)

target_link_libraries(${PROJECT_NAME} PRIVATE stm32h5_hal m)

target_link_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${MCU_LDSCRIPT}
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--no-warn-rwx-segments           # xóa dòng này nếu dùng binutils < 2.39
)

# Xuất .hex / .bin và in kích thước sau khi build
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
)
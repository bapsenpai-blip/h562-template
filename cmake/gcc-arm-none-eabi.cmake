# Toolchain file cho arm-none-eabi-gcc
set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ---------------------------------------------------------------------------
# Tìm trình biên dịch:
#   1. TOOLCHAIN_PREFIX nếu bạn tự đặt (vd: C:/.../bin/)
#   2. arm-none-eabi-gcc trong PATH
#   3. Bundle "gnu-tools-for-stm32" của extension STM32Cube for VS Code
#      (tự chọn phiên bản mới nhất)
# ---------------------------------------------------------------------------
set(TOOLCHAIN_PREFIX "" CACHE PATH "Thư mục bin của arm-none-eabi-gcc (có dấu / ở cuối)")

if(NOT TOOLCHAIN_PREFIX)
    find_program(_ARM_GCC_IN_PATH arm-none-eabi-gcc)
    if(NOT _ARM_GCC_IN_PATH)
        file(GLOB _ARM_GCC_CANDIDATES
            "$ENV{LOCALAPPDATA}/stm32cube/bundles/gnu-tools-for-stm32/*/bin/arm-none-eabi-gcc.exe"
            "$ENV{HOME}/.local/share/stm32cube/bundles/gnu-tools-for-stm32/*/bin/arm-none-eabi-gcc"
            "$ENV{HOME}/Library/Application Support/stm32cube/bundles/gnu-tools-for-stm32/*/bin/arm-none-eabi-gcc"
        )
        if(_ARM_GCC_CANDIDATES)
            list(SORT _ARM_GCC_CANDIDATES COMPARE NATURAL ORDER DESCENDING)
            list(GET _ARM_GCC_CANDIDATES 0 _ARM_GCC)
            get_filename_component(_ARM_GCC_DIR "${_ARM_GCC}" DIRECTORY)
            set(TOOLCHAIN_PREFIX "${_ARM_GCC_DIR}/" CACHE PATH "" FORCE)
            message(STATUS "Dùng toolchain từ bundle STM32Cube: ${_ARM_GCC_DIR}")
        else()
            message(FATAL_ERROR
                "Không tìm thấy arm-none-eabi-gcc.\n"
                "Cài bundle 'gnu-tools-for-stm32' trong STM32Cube Bundles Manager.")
        endif()
    endif()
endif()

if(CMAKE_HOST_WIN32)
    set(_EXE ".exe")
else()
    set(_EXE "")
endif()

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}arm-none-eabi-gcc${_EXE})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}arm-none-eabi-g++${_EXE})
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}arm-none-eabi-gcc${_EXE})
set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}arm-none-eabi-objcopy${_EXE})
set(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}arm-none-eabi-size${_EXE})

set(CMAKE_EXECUTABLE_SUFFIX_C   ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Cờ tối ưu theo build type
set(CMAKE_C_FLAGS_DEBUG          "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE        "-O2 -g0 -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL     "-Os -g0 -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3")
set(CMAKE_ASM_FLAGS_DEBUG        "-g3")

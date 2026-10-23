#!/usr/bin/env bash
# ==============================================================================
# Build Firmware Script for STM32F401CCU6 Self-Balancing Robot
# Target: Linux / macOS (ARM GNU Toolchain + CMake + Ninja / Make)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================="
echo " [BUILD] Compiling STM32F401CCU6 Balance Robot Firmware"
echo "=========================================================="

# 1. Check toolchain dependencies
if ! command -v cmake &> /dev/null; then
    echo "[ERROR] 'cmake' chua duoc cai dat. Vui long cai dat CMake 3.22+."
    exit 1
fi

if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "[ERROR] 'arm-none-eabi-gcc' chua duoc tim thay trong PATH."
    echo "        Vui long cai dat Arm GNU Toolchain (gcc-arm-none-eabi)."
    exit 1
fi

echo "[INFO] CMake: $(cmake --version | head -n 1)"
echo "[INFO] Cross-Compiler: $(arm-none-eabi-gcc --version | head -n 1)"

# 2. Configure build directory
BUILD_DIR="$SCRIPT_DIR/build/Debug"

echo "[INFO] Cau hinh CMake voi Toolchain file: cmake/gcc-arm-none-eabi.cmake..."
if cmake --list-presets | grep -q "Debug"; then
    cmake --preset Debug
else
    cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
fi

# 3. Build target
echo "[INFO] Dang bien dich ma nguon..."
cmake --build "$BUILD_DIR" -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# 4. Verify outputs
ELF_FILE="$BUILD_DIR/self_balancing_car.elf"
BIN_FILE="$BUILD_DIR/self_balancing_car.bin"
HEX_FILE="$BUILD_DIR/self_balancing_car.hex"

echo "----------------------------------------------------------"
if [ -f "$ELF_FILE" ] && [ -f "$BIN_FILE" ] && [ -f "$HEX_FILE" ]; then
    echo "[SUCCESS] Bien dich firmware thanh cong 100%!"
    echo "  -> ELF: $ELF_FILE ($(du -h "$ELF_FILE" | cut -f1))"
    echo "  -> BIN: $BIN_FILE ($(du -h "$BIN_FILE" | cut -f1))"
    echo "  -> HEX: $HEX_FILE ($(du -h "$HEX_FILE" | cut -f1))"
    echo "----------------------------------------------------------"
    echo "[FLASH HINT] Ban co the nap truc tiep bang STM32CubeProgrammer hoac OpenOCD:"
    echo "  st-flash write $BIN_FILE 0x08000000"
else
    echo "[ERROR] File build dich khong day du. Vui long kiem tra lai log loi phia tren."
    exit 1
fi
echo "=========================================================="

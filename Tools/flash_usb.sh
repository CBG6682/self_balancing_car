#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BIN_FILE="$PROJECT_ROOT/build/Debug/self_balancing_car.bin"

echo "=========================================================="
echo " [FLASH] Nạp Firmware STM32F401 Qua Cổng Type-C WeAct HID"
echo "=========================================================="

if [ ! -f "$BIN_FILE" ]; then
    echo "[ERROR] Không tìm thấy file nhị phân: $BIN_FILE"
    echo "Hãy chạy ./build_firmware.sh trước!"
    exit 1
fi

echo "[INFO] File nạp: $BIN_FILE ($(stat -c%s "$BIN_FILE") bytes)"

PYTHON_EXEC="$PROJECT_ROOT/venv/bin/python3"
if [ ! -f "$PYTHON_EXEC" ]; then
    PYTHON_EXEC="python3"
fi

# Check if running as root or if user has sudo
if [ "$EUID" -eq 0 ]; then
    "$PYTHON_EXEC" "$SCRIPT_DIR/flash_hid.py" "$BIN_FILE"
elif sudo -n true 2>/dev/null; then
    sudo "$PYTHON_EXEC" "$SCRIPT_DIR/flash_hid.py" "$BIN_FILE"
else
    # Try running directly; if permission denied, flash_hid.py will guide the user
    "$PYTHON_EXEC" "$SCRIPT_DIR/flash_hid.py" "$BIN_FILE" || {
        echo ""
        echo "[HƯỚNG DẪN] Cấp quyền nạp USB bằng lệnh sau (nhập mật khẩu máy tính):"
        echo "  sudo $PYTHON_EXEC $SCRIPT_DIR/flash_hid.py"
        echo "Hoặc cài đặt quyền USB vĩnh viễn không cần hỏi mật khẩu:"
        echo "  sudo $SCRIPT_DIR/setup_udev.sh"
    }
fi

#!/bin/bash
set -e

RULE_FILE="/etc/udev/rules.d/99-weact-studio.rules"

echo "=========================================================="
echo " Thiết lập quyền truy cập USB cho WeAct HID Bootloader"
echo "=========================================================="

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Vui lòng chạy lệnh này với quyền root: sudo ./Tools/setup_udev.sh"
    exit 1
fi

cat << 'EOF' > "$RULE_FILE"
# WeAct Studio HID Bootloader (0483:572a)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="572a", MODE="0666", TAG+="uaccess"
KERNEL=="hidraw*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="572a", MODE="0666", TAG+="uaccess"

# STM32 DFU Mode (0483:df11)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666", TAG+="uaccess"

# Standard USB-UART bridges (CH340, CP2102, FTDI)
KERNEL=="ttyUSB*", MODE="0666", TAG+="uaccess"
KERNEL=="ttyACM*", MODE="0666", TAG+="uaccess"
EOF

chmod 644 "$RULE_FILE"
udevadm control --reload-rules
udevadm trigger

echo "[SUCCESS] Đã cấu hình xong udev rule tại $RULE_FILE!"
echo "Quyền truy cập USB STM32 và Serial đã được mở cho tất cả người dùng."

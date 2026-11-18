#!/usr/bin/env bash
# ==============================================================================
# Master Pipeline Execution Script: STM32 Self-Balancing Robot
# Target: Linux / macOS
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PYTHON_EXEC="$SCRIPT_DIR/venv/bin/python3"

# ANSI Color formatting
BOLD_CYAN="\033[1;36m"
BOLD_GREEN="\033[1;32m"
BOLD_YELLOW="\033[1;33m"
BOLD_BLUE="\033[1;34m"
BOLD_RED="\033[1;31m"
RESET="\033[0m"

# Check virtual environment
if [ ! -f "$PYTHON_EXEC" ]; then
    echo -e "${BOLD_YELLOW}[WARN]${RESET} Chưa tìm thấy virtual environment tại venv/bin/python3."
    echo -e "${BOLD_BLUE}[INFO]${RESET} Đang tự động chạy ./setup.sh..."
    ./setup.sh
fi

echo -e "\n${BOLD_CYAN}╔══════════════════════════════════════════════════════════════════════╗"
echo -e "║   [PIPELINE] STM32 Self-Balancing Robot Toolchain                    ║"
echo -e "╚══════════════════════════════════════════════════════════════════════╝${RESET}\n"

run_self_test()       { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/system_self_test.py"; }
run_check_conn()      { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/check_connection.py"; }
run_sensor_calib()    { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/live_sensor_calib.py"; }
run_find_deadzone()   { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/find_deadzone.py"; }
run_find_offset()     { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/find_zero_balance.py"; }
run_live_tuning()     { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/live_balance_tuning.py"; }
run_tune_monitor()    { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/esp32_tune_monitor.py"; }
run_record_session()  { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/record_session.py"; }
run_sim_lqr()         { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/sim_and_init_pid.py"; }
run_auto_tune()       { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/auto_tune_pid.py"; }
rebuild_firmware()    { ./build_firmware.sh; }
run_flash_stlink()    { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/flash_stlink.py" "$SCRIPT_DIR/build/Debug/self_balancing_car.bin"; }
run_flash_hid()       { "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/flash_hid.py" "$SCRIPT_DIR/build/Debug/self_balancing_car.bin"; }

case "${1:-}" in
    --selftest|--diag|-d) run_self_test; exit 0 ;;
    --check|-c)           run_check_conn; exit 0 ;;
    --calib|-s)           run_sensor_calib; exit 0 ;;
    --deadzone|-dz)       run_find_deadzone; exit 0 ;;
    --offset|-o)          run_find_offset; exit 0 ;;
    --tune|-t)            run_live_tuning; exit 0 ;;
    --monitor|-m)         run_tune_monitor; exit 0 ;;
    --record|-r)          run_record_session; exit 0 ;;
    --sim)                run_sim_lqr; exit 0 ;;
    --autotune)           run_auto_tune; exit 0 ;;
    --build|-b)           rebuild_firmware; exit 0 ;;
    --stlink|-st)         run_flash_stlink; exit 0 ;;
    --flash|-f)           run_flash_hid; exit 0 ;;
    --help|-h)
        echo "Cách sử dụng: ./run_pipeline.sh [OPTION]"
        echo "Options:"
        echo "  --selftest, -d   Kiểm tra tổng thể từng linh kiện (system_self_test.py)"
        echo "  --check, -c      Kiểm tra kết nối USB-UART / ESP32 (check_connection.py)"
        echo "  --calib, -s      Hiệu chuẩn cảm biến MPU6050 & Encoder (live_sensor_calib.py)"
        echo "  --deadzone, -dz  Đo vùng chết động cơ trái/phải (find_deadzone.py)"
        echo "  --offset, -o     Đo góc cân bằng cơ học Offset (find_zero_balance.py)"
        echo "  --tune, -t       Tinh chỉnh PID trực tiếp (live_balance_tuning.py)"
        echo "  --monitor, -m    Giám sát & tinh chỉnh qua ESP32 Bridge (esp32_tune_monitor.py)"
        echo "  --record, -r     Ghi phiên dữ liệu telemetry 100Hz (record_session.py)"
        echo "  --sim            Mô phỏng vật lý & khởi tạo LQR PID (sim_and_init_pid.py)"
        echo "  --autotune       Tối ưu hóa PID tự động (auto_tune_pid.py)"
        echo "  --build, -b      Biên dịch firmware STM32 (build_firmware.sh)"
        echo "  --stlink, -st    Nạp firmware qua ST-Link SWD (flash_stlink.py)"
        echo "  --flash, -f      Nạp firmware qua USB HID Bootloader (flash_hid.py)"
        exit 0
        ;;
esac

echo "Chọn công cụ thực thi:"
echo "  1) [KHUYẾN NGHỊ] Kiểm tra tổng thể phần cứng (Tools/system_self_test.py)"
echo "  2) Kiểm tra kết nối USB-UART / ESP32 Bridge  (Tools/check_connection.py)"
echo "  3) Hiệu chuẩn cảm biến MPU6050 & Encoder     (Tools/live_sensor_calib.py)"
echo "  4) Đo Deadzone từng động cơ GA25             (Tools/find_deadzone.py)"
echo "  5) Tìm góc Offset cân bằng cơ khí            (Tools/find_zero_balance.py)"
echo "  6) Tinh chỉnh PID trực tiếp                  (Tools/live_balance_tuning.py)"
echo "  7) Giám sát & gửi lệnh qua ESP32 Bridge      (Tools/esp32_tune_monitor.py)"
echo "  8) Ghi dữ liệu telemetry 100Hz (CSV)         (Tools/record_session.py)"
echo "  9) Mô phỏng vật lý & khởi tạo LQR PID        (Tools/sim_and_init_pid.py)"
echo " 10) Tối ưu hóa PID tự động (ARX/ITAE)         (Tools/auto_tune_pid.py)"
echo " 11) Biên dịch firmware STM32                  (build_firmware.sh)"
echo " 12) Nạp firmware qua ST-Link V2/V3 (SWD)      (Tools/flash_stlink.py)"
echo " 13) Nạp firmware qua USB HID Bootloader       (Tools/flash_hid.py)"
echo "  0) Thoát"
echo "----------------------------------------------------------"
read -r -p "Nhập lựa chọn [0-13]: " choice

case "$choice" in
    1)  run_self_test ;;
    2)  run_check_conn ;;
    3)  run_sensor_calib ;;
    4)  run_find_deadzone ;;
    5)  run_find_offset ;;
    6)  run_live_tuning ;;
    7)  run_tune_monitor ;;
    8)  run_record_session ;;
    9)  run_sim_lqr ;;
    10) run_auto_tune ;;
    11) rebuild_firmware ;;
    12) run_flash_stlink ;;
    13) run_flash_hid ;;
    0)  echo "Đã thoát."; exit 0 ;;
    *)  echo -e "${BOLD_RED}[ERROR] Lựa chọn không hợp lệ!${RESET}"; exit 1 ;;
esac

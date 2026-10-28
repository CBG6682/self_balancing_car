#!/usr/bin/env bash
# ==============================================================================
# Master Pipeline Execution Script: Self-Balancing Robot
# Runs Steps 2 -> 6 (Simulation, LQR, Data Acq, System ID, Auto-tune, Verification)
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
echo -e "║   [PIPELINE] STM32 Self-Balancing Robot Automated Toolchain          ║"
echo -e "╚══════════════════════════════════════════════════════════════════════╝${RESET}\n"

run_sim_lqr() {
    echo -e "\n${BOLD_BLUE}>>> [1/4] Mô Phỏng Vật Lý & Khởi Tạo LQR PID...${RESET}"
    "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/sim_and_init_pid.py"
}

run_data_acq() {
    echo -e "\n${BOLD_BLUE}>>> [2/4] Thu Thập Dữ Liệu / Sinh Dữ Liệu Giả Lập...${RESET}"
    "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/data_acquisition.py"
}

run_auto_tune() {
    echo -e "\n${BOLD_BLUE}>>> [3/4] Nhận Dạng Hệ Thống & Tối Ưu Hóa PID (Nelder-Mead ITAE)...${RESET}"
    "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/auto_tune_pid.py"
}

run_test_seq() {
    echo -e "\n${BOLD_BLUE}>>> [4/4] Kiểm Thử Kịch Bản Điều Khiển Tự Động & Vẽ Đồ Thị...${RESET}"
    "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/test_control_sequence.py"
}

rebuild_firmware() {
    echo -e "\n${BOLD_BLUE}>>> [AUTO-BUILD] Biên dịch lại firmware với thông số PID mới...${RESET}"
    ./build_firmware.sh
}

run_all() {
    run_sim_lqr
    run_data_acq
    run_auto_tune
    run_test_seq
    rebuild_firmware
    echo -e "\n${BOLD_GREEN}══════════════════════════════════════════════════════════════════════"
    echo -e "       [HOÀN THÀNH TRỌN VẸN CHUỖI PIPELINE AUTO-TUNING]               "
    echo -e "  ✔ Header C đã được cập nhật : Modules/Inc/pid_AutoConfig.h           "
    echo -e "  ✔ Firmware đã được biên dịch: build/Debug/self_balancing_car.bin    "
    echo -e "  ✔ Đồ thị kiểm thử đã được lưu: Tools/test_result_plot.png           "
    echo -e "══════════════════════════════════════════════════════════════════════${RESET}\n"
}

# Command line arguments parsing
run_hardware_check() {
    "$PYTHON_BIN" Tools/hardware_check.py
}

run_check_connection() {
    "$PYTHON_BIN" Tools/check_connection.py
}

run_motor_test() {
    "$PYTHON_BIN" Tools/motor_test_interactive.py
}

run_sensor_test() {
    "$PYTHON_BIN" Tools/sensor_test_live.py
}

run_bridge_test() {
    "$PYTHON_BIN" Tools/test_bridge_live.py
}

run_flash_hid() {
    "$PYTHON_BIN" Tools/flash_hid.py build/Debug/self_balancing_car.bin
}

run_flash_stlink() {
    "$PYTHON_BIN" Tools/flash_stlink.py build/Debug/self_balancing_car.bin
}

run_self_test() {
    "$PYTHON_BIN" Tools/system_self_test.py
}

if [ "$1" == "--all" ] || [ "$1" == "-a" ]; then
    run_all
    exit 0
elif [ "$1" == "--selftest" ] || [ "$1" == "--diag" ] || [ "$1" == "-d" ]; then
    run_self_test
    exit 0
elif [ "$1" == "--check" ] || [ "$1" == "-c" ]; then
    run_check_connection
    exit 0
elif [ "$1" == "--hardware" ] || [ "$1" == "-hw" ]; then
    run_hardware_check
    exit 0
elif [ "$1" == "--motor" ] || [ "$1" == "-m" ]; then
    run_motor_test
    exit 0
elif [ "$1" == "--sensor" ] || [ "$1" == "-s" ]; then
    run_sensor_test
    exit 0
elif [ "$1" == "--bridge" ] || [ "$1" == "-b" ]; then
    run_bridge_test
    exit 0
elif [ "$1" == "--loopback" ] || [ "$1" == "-lb" ]; then
    "$PYTHON_BIN" Tools/test_stm32_loopback.py
    exit 0
elif [ "$1" == "--flash" ] || [ "$1" == "-f" ]; then
    run_flash_hid
    exit 0
elif [ "$1" == "--stlink" ] || [ "$1" == "-st" ]; then
    run_flash_stlink
    exit 0
elif [ "$1" == "--sim" ] || [ "$1" == "--step2" ]; then
    run_sim_lqr
    exit 0
elif [ "$1" == "--data" ] || [ "$1" == "--step4" ]; then
    run_data_acq
    exit 0
elif [ "$1" == "--tune" ] || [ "$1" == "--step5" ]; then
    run_auto_tune
    rebuild_firmware
    exit 0
elif [ "$1" == "--test" ] || [ "$1" == "--step6" ]; then
    run_test_seq
    exit 0
elif [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "Cách sử dụng: ./run_pipeline.sh [OPTION]"
    echo "Options:"
    echo "  --selftest, -d   Kiểm tra TỔNG THỂ từng linh kiện có xác nhận (system_self_test.py)"
    echo "  --all, -a        Chạy toàn bộ quy trình từ Mô phỏng đến Kiểm thử và build firmware"
    echo "  --motor, -m      Kiểm tra động cơ TB6612 tương tác (motor_test_interactive.py)"
    echo "  --sensor, -s     Bảng theo dõi cảm biến IMU MPU6050 & Encoder (sensor_test_live.py)"
    echo "  --bridge, -b     Kiểm tra thông tuyến UART thời gian thực (test_bridge_live.py)"
    echo "  --flash, -f      Nạp firmware STM32 qua USB HID Bootloader (flash_hid.py)"
    echo "  --stlink, -st    Nạp firmware STM32 qua mạch nạp ST-Link SWD (flash_stlink.py)"
    echo "  --hardware, -hw  Kiểm tra phần cứng & Tune xe trực tiếp (Live check & Auto-tune)"
    echo "  --check, -c      Chẩn đoán & Lắng nghe kết nối USB-UART thời gian thực (check_connection.py)"
    echo "  --sim            Mô phỏng & Khởi tạo LQR PID từ robot_config.yaml"
    echo "  --data           Thu thập telemetry từ UART hoặc sinh mock CSV"
    echo "  --tune           Nhận dạng ARX & Tối ưu hóa PID (Nelder-Mead ITAE)"
    echo "  --test           Kiểm thử kịch bản điều khiển tự động & xuất đồ thị"
    echo "  (Không đối số): Hiện menu lựa chọn tương tác"
    exit 0
fi

# Interactive menu if no arguments provided
echo "Chọn chế độ chạy:"
echo "  1) [KHUYẾN NGHỊ] Kiểm tra TỔNG THỂ từng linh kiện & kết nối (Tools/system_self_test.py)"
echo "  2) Chạy TOÀN BỘ quy trình Auto-Tuning (Mô phỏng -> Thu thập -> Tối ưu -> Build)"
echo "  3) Mô phỏng Vật lý & Khởi tạo LQR PID"
echo "  4) Thu thập / Sinh dữ liệu telemetry (CSV)"
echo "  5) Nhận dạng hệ thống ARX & Tối ưu hóa PID"
echo "  6) Kiểm thử kịch bản điều khiển & Vẽ đồ thị phản hồi"
echo "  7) Chỉ biên dịch firmware STM32 (build_firmware.sh)"
echo "  8) Kiểm tra phần cứng & Tune xe thực tế (Tools/hardware_check.py)"
echo "  9) Kiểm tra động cơ tương tác (Tools/motor_test_interactive.py)"
echo " 10) Bảng theo dõi cảm biến IMU & Encoder (Tools/sensor_test_live.py)"
echo " 11) Kiểm tra thông tuyến UART ESP32 <-> STM32 (Tools/test_bridge_live.py)"
echo " 12) Nạp firmware STM32 qua USB HID Type-C (Tools/flash_hid.py)"
echo " 13) Nạp firmware STM32 qua ST-Link V2/V3 (Tools/flash_stlink.py)"
echo " 14) Chẩn đoán & Lắng nghe kết nối USB-UART (Tools/check_connection.py)"
echo "  0) Thoát"
echo "----------------------------------------------------------"
read -r -p "Nhập lựa chọn [1-14, 0]: " choice

case "$choice" in
    1) run_self_test ;;
    2) run_all ;;
    3) run_sim_lqr ;;
    4) run_data_acq ;;
    5) run_auto_tune; rebuild_firmware ;;
    6) run_test_seq ;;
    7) rebuild_firmware ;;
    8) run_hardware_check ;;
    9) run_motor_test ;;
    10) run_sensor_test ;;
    11) run_bridge_test ;;
    12) run_flash_hid ;;
    13) run_flash_stlink ;;
    14) run_check_connection ;;
    0) echo "Đã thoát."; exit 0 ;;
    *) echo -e "${BOLD_RED}[ERROR] Lựa chọn không hợp lệ!${RESET}"; exit 1 ;;
esac

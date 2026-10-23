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
if [ "$1" == "--all" ] || [ "$1" == "-a" ]; then
    run_all
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
    echo "  --all, -a     Chạy toàn bộ quy trình từ Mô phỏng đến Kiểm thử và build firmware"
    echo "  --sim         Mô phỏng & Khởi tạo LQR PID từ robot_config.yaml"
    echo "  --data        Thu thập telemetry từ UART hoặc sinh mock CSV"
    echo "  --tune        Nhận dạng ARX & Tối ưu hóa PID (Nelder-Mead ITAE)"
    echo "  --test        Kiểm thử kịch bản điều khiển tự động & xuất đồ thị"
    echo "  (Không đối số): Hiện menu lựa chọn tương tác"
    exit 0
fi

# Interactive menu if no arguments provided
echo "Chọn chế độ chạy:"
echo "  1) Chạy TOÀN BỘ quy trình (Mô phỏng -> Thu thập -> Tối ưu -> Kiểm thử -> Rebuild Firmware)"
echo "  2) Mô phỏng Vật lý & Khởi tạo LQR PID"
echo "  3) Thu thập / Sinh dữ liệu telemetry (CSV)"
echo "  4) Nhận dạng hệ thống ARX & Tối ưu hóa PID"
echo "  5) Kiểm thử kịch bản điều khiển & Vẽ đồ thị phản hồi"
echo "  6) Chỉ biên dịch firmware STM32 (build_firmware.sh)"
echo "  0) Thoát"
echo "----------------------------------------------------------"
read -r -p "Nhập lựa chọn [1-6, 0]: " choice

case "$choice" in
    1)
        run_all
        ;;
    2)
        run_sim_lqr
        ;;
    3)
        run_data_acq
        ;;
    4)
        run_auto_tune
        rebuild_firmware
        ;;
    5)
        run_test_seq
        ;;
    6)
        rebuild_firmware
        ;;
    0)
        echo "Đã thoát."
        exit 0
        ;;
    *)
        echo -e "${BOLD_RED}[ERROR] Lựa chọn không hợp lệ!${RESET}"
        exit 1
        ;;
esac

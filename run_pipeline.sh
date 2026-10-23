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

# Check virtual environment
if [ ! -f "$PYTHON_EXEC" ]; then
    echo "[WARN] Chua tim thay virtual environment tai venv/bin/python3."
    echo "[INFO] Dang tu dong chay ./setup.sh..."
    ./setup.sh
fi

echo "=========================================================="
echo " [PIPELINE] STM32 Self-Balancing Robot Automated Toolchain"
echo "=========================================================="

run_step2() {
    echo -e "\n>>> [1/4] CHAY STEP 2: Mo phong Vat ly & Khoi tao LQR PID..."
    "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/step2_sim_and_init_pid.py"
}

run_step4() {
    echo -e "\n>>> [2/4] CHAY STEP 4: Thu Thap Du Lieu / Sinh Giu Lieu Gia Lap..."
    "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/step4_data_acquisition.py"
}

run_step5() {
    echo -e "\n>>> [3/4] CHAY STEP 5: Nhan Dang He Thong & Toi Uu Hoa PID (Nelder-Mead ITAE)..."
    "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/step5_auto_tune_pid.py"
}

run_step6() {
    echo -e "\n>>> [4/4] CHAY STEP 6: Kiem Thu Kich Ban Dieu Khien Tu Dong & Ve Do Thi..."
    "$PYTHON_EXEC" "$SCRIPT_DIR/Tools/step6_test_control_sequence.py"
}

rebuild_firmware() {
    echo -e "\n>>> [AUTO-BUILD] Bien dich lai firmware voi thong so PID moi..."
    ./build_firmware.sh
}

run_all() {
    run_step2
    run_step4
    run_step5
    run_step6
    rebuild_firmware
    echo -e "\n=========================================================="
    echo " [HOAN THANH TRON VEN CHUOI PIPELINE AUTO-TUNING] "
    echo "  - Header C da duoc cap nhat: Modules/Inc/pid_AutoConfig.h"
    echo "  - Firmware da duoc bien dich: build/Debug/self_balancing_car.bin"
    echo "  - Do thi kiem thu da duoc luu: Tools/test_result_plot.png"
    echo "=========================================================="
}

# Command line arguments parsing
if [ "$1" == "--all" ] || [ "$1" == "-a" ]; then
    run_all
    exit 0
elif [ "$1" == "--step2" ]; then
    run_step2
    exit 0
elif [ "$1" == "--step4" ]; then
    run_step4
    exit 0
elif [ "$1" == "--step5" ]; then
    run_step5
    rebuild_firmware
    exit 0
elif [ "$1" == "--step6" ]; then
    run_step6
    exit 0
elif [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "Cach su dung: ./run_pipeline.sh [OPTION]"
    echo "Options:"
    echo "  --all, -a     Chay toan bo quy trinh tu Step 2 den Step 6 va build firmware"
    echo "  --step2       Chi chay Step 2: Khoi tao LQR PID tu robot_config.yaml"
    echo "  --step4       Chi chay Step 4: Thu thap telemetry tu UART hoac sinh mock CSV"
    echo "  --step5       Chi chay Step 5: Nhan dang ARX & Toi uu PID"
    echo "  --step6       Chi chay Step 6: Kiem thu kịch ban tu dong & xuat do thi"
    echo "  (Khong doi so): Hien menu lua chon tuong tac"
    exit 0
fi

# Interactive menu if no arguments provided
echo "Chon che do chay:"
echo "  1) Chay TOAN BO quy trinh (Step 2 -> 4 -> 5 -> 6 -> Rebuild Firmware)"
echo "  2) Chay Step 2: Mo phong & Tinh toan LQR PID khoi diem"
echo "  3) Chay Step 4: Thu thap / Sinh du lieu telemetry (CSV)"
echo "  4) Chay Step 5: Nhan dang he thong ARX & Toi uu hoa PID"
echo "  5) Chay Step 6: Kiem thu kịch ban dieu khien & Ve do thi phan hoi"
echo "  6) Chi bien dich firmware STM32 (build_firmware.sh)"
echo "  0) Thoat"
echo "----------------------------------------------------------"
read -r -p "Nhap lua chon [1-6, 0]: " choice

case "$choice" in
    1)
        run_all
        ;;
    2)
        run_step2
        ;;
    3)
        run_step4
        ;;
    4)
        run_step5
        rebuild_firmware
        ;;
    5)
        run_step6
        ;;
    6)
        rebuild_firmware
        ;;
    0)
        echo "Da thoat."
        exit 0
        ;;
    *)
        echo "[ERROR] Lua chon khong hop le!"
        exit 1
        ;;
esac

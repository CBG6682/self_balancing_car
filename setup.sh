#!/usr/bin/env bash
# ==============================================================================
# Setup Environment for Self-Balancing Robot Tools
# Target: Linux / macOS
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================="
echo " [SETUP] Self-Balancing Robot - Environment Installation"
echo "=========================================================="

# 1. Check Python 3
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] python3 chua duoc cai dat tren he thong. Vui long cai dat Python 3.8+."
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
echo "[INFO] Phat hien Python version: $PYTHON_VERSION"

# 2. Setup Virtual Environment
VENV_DIR="$SCRIPT_DIR/venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] Dang khoi tao virtual environment tai: $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
else
    echo "[INFO] Virtual environment da ton tai tai: $VENV_DIR"
fi

# 3. Upgrade pip and install requirements
echo "[INFO] Cai dat cac thu vien phan tich & mo phong (Tools/requirements.txt)..."
"$VENV_DIR/bin/python3" -m pip install --upgrade pip
"$VENV_DIR/bin/python3" -m pip install -r "$SCRIPT_DIR/Tools/requirements.txt"

# 4. Set execution permissions for shell scripts
chmod +x "$SCRIPT_DIR/setup.sh" || true
chmod +x "$SCRIPT_DIR/build_firmware.sh" || true
chmod +x "$SCRIPT_DIR/run_pipeline.sh" || true
chmod +x "$SCRIPT_DIR"/Tools/*.py || true

echo "=========================================================="
echo " [SETUP COMPLETED] Moi truong Python da san sang!"
echo " Ban co the chay ./build_firmware.sh de bien dich firmware"
echo " hoac ./run_pipeline.sh de chay chuoi toi uu hoa PID."
echo "=========================================================="

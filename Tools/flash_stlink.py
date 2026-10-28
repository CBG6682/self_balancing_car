#!/usr/bin/env python3
"""
ST-Link Flasher for STM32F401CCU6 Self-Balancing Robot
Supports ST-Link V2 / V3 / V2-1 / clones.
Auto-detects available backend: st-flash, STM32_Programmer_CLI, pyocd, openocd.
Cross-platform: Linux, Windows, macOS.
"""

import os
import sys
import shutil
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, SCRIPT_DIR)
import logger_utils as logger

FLASH_BASE_ADDR = "0x08004000"
DEFAULT_BIN = os.path.join(PROJECT_ROOT, "build", "Debug", "self_balancing_car.bin")
DEFAULT_ELF = os.path.join(PROJECT_ROOT, "build", "Debug", "self_balancing_car.elf")
DEFAULT_HEX = os.path.join(PROJECT_ROOT, "build", "Debug", "self_balancing_car.hex")


def print_stlink_pinout():
    print("\n" + "=" * 65)
    print(" NẠP FIRMWARE STM32 QUA MẠCH NẠP ST-LINK (SWD INTERFACE)")
    print(" Target: STM32F401CCU6 BlackPill | Địa chỉ Flash: " + FLASH_BASE_ADDR)
    print("=" * 65)
    print(" SƠ ĐỒ ĐẤU DÂY ST-LINK V2 <---> BLACKPILL (Hàng 4 chân đuôi mạch):")
    print("  • ST-Link SWDIO  <---> BlackPill DIO (PA13)")
    print("  • ST-Link SWCLK  <---> BlackPill CLK (PA14)")
    print("  • ST-Link GND    <---> BlackPill GND")
    print("  • ST-Link 3.3V   <---> BlackPill 3.3V (hoặc nuôi qua cổng USB)")
    print("=" * 65 + "\n")


def check_and_build(bin_path):
    if not os.path.exists(bin_path):
        logger.warn(f"Chưa thấy file nhị phân '{os.path.basename(bin_path)}'.")
        logger.info("Đang tự động biên dịch firmware qua Ninja/CMake...")
        build_script = os.path.join(PROJECT_ROOT, "build_firmware.sh" if sys.platform != "win32" else "build_firmware.bat")
        if os.path.exists(build_script):
            subprocess.run([build_script], cwd=PROJECT_ROOT)
        else:
            subprocess.run(["ninja", "-C", "build/Debug"], cwd=PROJECT_ROOT)
    if not os.path.exists(bin_path):
        logger.error(f"Không thể tìm thấy file '{bin_path}' sau khi build!")
        sys.exit(1)


def flash_with_st_flash(bin_path):
    cmd = ["st-flash", "write", bin_path, FLASH_BASE_ADDR]
    logger.info(f"Đang thực thi: {' '.join(cmd)}")
    res = subprocess.run(cmd)
    return res.returncode == 0


def flash_with_stm32_programmer_cli(target_file):
    # Try .hex first (contains absolute addresses), then .bin with address
    if target_file.endswith(".bin"):
        cmd = ["STM32_Programmer_CLI", "-c", "port=SWD", "-w", target_file, FLASH_BASE_ADDR, "-v", "-rst"]
    else:
        cmd = ["STM32_Programmer_CLI", "-c", "port=SWD", "-w", target_file, "-v", "-rst"]
    logger.info(f"Đang thực thi: {' '.join(cmd)}")
    res = subprocess.run(cmd)
    return res.returncode == 0


def flash_with_pyocd(target_file):
    pyocd_exec = shutil.which("pyocd") or os.path.join(PROJECT_ROOT, "venv", "bin" if sys.platform != "win32" else "Scripts", "pyocd")
    if target_file.endswith(".bin"):
        cmd = [pyocd_exec, "flash", "--no-wait", "-t", "stm32f401cc", "--base-address", FLASH_BASE_ADDR, target_file]
    else:
        cmd = [pyocd_exec, "flash", "--no-wait", "-t", "stm32f401cc", target_file]
    logger.info(f"Đang thực thi: {' '.join(cmd)}")
    res = subprocess.run(cmd)
    return res.returncode == 0


def flash_with_openocd(target_file):
    cfg_cmd = f"program {target_file} {FLASH_BASE_ADDR} verify reset exit"
    cmd = ["openocd", "-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg", "-c", cfg_cmd]
    logger.info(f"Đang thực thi: {' '.join(cmd)}")
    res = subprocess.run(cmd)
    return res.returncode == 0


def main():
    target_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BIN
    print_stlink_pinout()
    check_and_build(target_path)

    file_size = os.path.getsize(target_path)
    logger.info(f"File firmware: {os.path.basename(target_path)} ({file_size:,} bytes)")

    # 1. Detect which ST-Link backend tool is installed
    backends = []
    if shutil.which("st-flash"):
        backends.append(("st-flash", flash_with_st_flash, target_path))
    if shutil.which("STM32_Programmer_CLI"):
        backends.append(("STM32_Programmer_CLI", flash_with_stm32_programmer_cli, DEFAULT_HEX if os.path.exists(DEFAULT_HEX) else target_path))
    
    # Check pyocd in PATH or inside venv
    venv_pyocd = os.path.join(PROJECT_ROOT, "venv", "bin" if sys.platform != "win32" else "Scripts", "pyocd" + (".exe" if sys.platform == "win32" else ""))
    if shutil.which("pyocd") or os.path.exists(venv_pyocd):
        backends.append(("pyocd", flash_with_pyocd, target_path))

    if shutil.which("openocd"):
        backends.append(("openocd", flash_with_openocd, target_path))

    if not backends:
        logger.error("Không tìm thấy công cụ nạp ST-Link nào trên hệ thống!")
        print("\n[HƯỚNG DẪN CÀI ĐẶT]:")
        print("  • Trên Linux:   sudo pacman -S stlink (hoặc sudo apt install stlink-tools)")
        print("  • Trên Windows: Cài STM32CubeProgrammer hoặc chạy: pip install pyocd")
        print("  • Hoặc dùng script nạp USB có sẵn: ./run_pipeline.sh --flash")
        sys.exit(1)

    # 2. Execute flashing with preferred backend
    success = False
    for name, func, path in backends:
        logger.info(f"Sử dụng công cụ nạp: {logger.BOLD_CYAN}{name}{logger.RESET}")
        try:
            if func(path):
                success = True
                break
            else:
                logger.warn(f"Công cụ '{name}' không nạp được. Đang thử công cụ dự phòng khác...")
        except Exception as e:
            logger.warn(f"Lỗi khi chạy '{name}': {e}")

    if success:
        print()
        logger.success("NẠP FIRMWARE QUA ST-LINK THÀNH CÔNG RỰC RỠ!")
        print(f"  ✔ Địa chỉ nạp: {FLASH_BASE_ADDR} (Bảo toàn nguyên vẹn WeAct HID Bootloader)")
        print("  ✔ STM32 đã được Reset và bắt đầu chạy chương trình.")
    else:
        print()
        logger.error("Nạp firmware qua ST-Link thất bại!")
        print("  [GỢI Ý KHẮC PHỤC]:")
        print("  1. Kiểm tra 4 dây cắm: DIO (PA13), CLK (PA14), GND, 3.3V.")
        print("  2. Nhấn giữ nút NRST trên BlackPill, bấm nạp rồi nhả NRST (Connect Under Reset).")
        print("  3. Nếu mạch đang ở chế độ ngủ (Sleep) hoặc chân SWD bị vô hiệu hóa, gạt Boot0 lên 3.3V rồi nạp lại.")
        sys.exit(1)


if __name__ == "__main__":
    main()

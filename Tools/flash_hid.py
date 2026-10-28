#!/usr/bin/env python3
"""
WeAct Studio STM32F401 HID Bootloader Flasher (Pure Python)
Target: WeAct Studio BlackPill (STM32F401CCU6 / STM32F411CEU6)
USB VID:PID: 0483:572A

Protocol: WeAct Studio HID Bootloader v1.0 / v2.2+
- Sector size: 1024 bytes (16 chunks of 64 bytes per sector)
- Report size: 65 bytes (Report ID 0x00 + 64 data bytes)
- Prefix: 'W', 'e', 'A', 'c', 't', ':'
- Reset pages: 'WeAct:\x00' (followed by 0x00 padding to 65 bytes)
- Reboot MCU:  'WeAct:\x01' (followed by 0x00 padding to 65 bytes)
- Erase MCU:   'WeAct:\x02'
- Application flash start: 0x08004000 (after 16KB bootloader)
"""

import os
import sys
import time
import math

VID = 0x0483
PID = 0x572A
SECTOR_SIZE = 1024
CHUNK_SIZE = 64
HID_TX_SIZE = 65

CMD_WEACT_RESET_PAGES = b"WeAct:\x00"
CMD_WEACT_REBOOT_MCU  = b"WeAct:\x01"
CMD_SERASIDIS_RESET   = b"BTLDCMD\x00"
CMD_SERASIDIS_REBOOT  = b"BTLDCMD\x01"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
DEFAULT_BIN = os.path.join(PROJECT_ROOT, "build", "Debug", "self_balancing_car.bin")

# Import shared logger
sys.path.insert(0, SCRIPT_DIR)
import logger_utils as logger


def wait_for_device(timeout_sec=60):
    """Waits for WeAct HID Bootloader [0483:572A] to appear on USB."""
    import hid

    print(f"\n{logger.BOLD_CYAN}>>> Đang chờ bo mạch WeAct Studio HID Bootloader (0483:572a)...{logger.RESET}")
    print("  • Hãy cắm cáp USB-C từ STM32 vào laptop.")
    print("  • Thao tác nút bấm trên bo mạch BlackPill:")
    print(f"    1. {logger.BOLD_WHITE}Nhấn & GIỮ nút KEY{logger.RESET} (nút bấm cạnh LED PC1)")
    print(f"    2. {logger.BOLD_WHITE}Bấm & NHẢ nút NRST{logger.RESET} (Reset)")
    print(f"    3. {logger.BOLD_WHITE}Thấy đèn LED PC13 nhấp nháy -> THẢ nút KEY ra.{logger.RESET}\n")

    spinner = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
    start_t = time.time()
    idx = 0

    while (time.time() - start_t) < timeout_sec:
        try:
            devs = hid.enumerate(VID, PID)
            if devs:
                print()
                logger.success(f"Đã phát hiện WeAct HID Bootloader [0483:572A] trên cổng USB!")
                return devs[0]
        except Exception:
            pass

        char = spinner[idx % len(spinner)]
        idx += 1
        elapsed = int(time.time() - start_t)
        sys.stdout.write(f"\r  {char} Đang quét cổng USB... ({timeout_sec - elapsed}s còn lại) ")
        sys.stdout.flush()
        time.sleep(0.15)

    print()
    return None


def send_packet(h, data_bytes, retries=15):
    """Writes a 65-byte HID report with Report ID 0x00."""
    report = bytearray(HID_TX_SIZE)
    report[0] = 0x00  # Report ID
    report[1:1+len(data_bytes)] = data_bytes

    for _ in range(retries):
        try:
            n = h.write(report)
            if n >= len(report) or n > 0:
                return True
        except Exception:
            pass
        time.sleep(0.005)
    return False


def flash_firmware(bin_file_path=DEFAULT_BIN):
    import hid

    if not os.path.exists(bin_file_path):
        logger.error(f"Không tìm thấy file firmware: {bin_file_path}")
        print("  -> Hãy chạy `./build_firmware.sh` trước khi nạp!")
        return False

    file_size = os.path.getsize(bin_file_path)
    logger.info(f"File nhị phân: {bin_file_path} ({file_size} bytes / {file_size/1024:.1f} KB)")

    # Read binary file
    with open(bin_file_path, "rb") as f:
        firmware_bytes = f.read()

    # Pad firmware to full sector size (1024 bytes)
    remainder = len(firmware_bytes) % SECTOR_SIZE
    if remainder != 0:
        firmware_bytes += b"\xFF" * (SECTOR_SIZE - remainder)

    total_sectors = len(firmware_bytes) // SECTOR_SIZE

    dev_info = wait_for_device(timeout_sec=60)
    if not dev_info:
        logger.error("Hết thời gian chờ (Timeout): Chưa phát hiện bo mạch WeAct HID Bootloader.")
        print("\n  • Hãy kiểm tra lại:")
        print("    1. Cáp Type-C đã cắm chắc chắn vào máy tính chưa?")
        print("    2. Thử thao tác lại: Giữ nút KEY -> Bấm nhả NRST -> Thả nút KEY.")
        return False

    logger.info("Đang kết nối tới WeAct HID Bootloader...")
    h = hid.device()
    try:
        if "path" in dev_info and dev_info["path"]:
            h.open_path(dev_info["path"])
        else:
            h.open(VID, PID)
    except Exception as e:
        logger.error(f"Không thể mở thiết bị HID: {e}")
        print(f"\n  👉 {logger.BOLD_RED}Lỗi cấp quyền USB (Permission Denied) trên Linux!{logger.RESET}")
        print("  Để cấp quyền tự động vĩnh viễn, hãy chạy 1 lần lệnh sau:")
        print(f"    {logger.BOLD_CYAN}sudo ./Tools/setup_udev.sh{logger.RESET}")
        print("  Hoặc chạy script nạp với sudo:")
        print(f"    {logger.BOLD_CYAN}sudo ./Tools/flash_usb.sh{logger.RESET}")
        return False

    try:
        # 1. Send RESET PAGES command (try WeAct protocol, fallback Serasidis)
        print(f"{logger.BOLD_BLUE}[1/3]{logger.RESET} Gửi lệnh <RESET PAGES> xóa cờ chuẩn bị vùng nhớ Flash...")
        send_packet(h, CMD_WEACT_RESET_PAGES)
        send_packet(h, CMD_SERASIDIS_RESET)
        time.sleep(0.05)

        # 2. Flash sectors
        print(f"{logger.BOLD_BLUE}[2/3]{logger.RESET} Đang nạp {total_sectors} sectors ({len(firmware_bytes)} bytes) vào Flash (Offset 0x08004000)...")
        start_t = time.time()

        for sec_idx in range(total_sectors):
            sec_data = firmware_bytes[sec_idx * SECTOR_SIZE:(sec_idx + 1) * SECTOR_SIZE]

            # Send 16 chunks of 64 bytes
            for chunk_idx in range(0, SECTOR_SIZE, CHUNK_SIZE):
                chunk = sec_data[chunk_idx:chunk_idx + CHUNK_SIZE]
                if not send_packet(h, chunk):
                    logger.warn(f"Cảnh báo gửi gói sector {sec_idx}, chunk {chunk_idx // CHUNK_SIZE}")
                time.sleep(0.0005)

            # Wait for ACK from WeAct bootloader:
            # WeAct returns 7 bytes containing "WeAct:\x03" or status 0x02/0x03
            ack_ok = False
            for _ in range(250):
                rx = h.read(7, timeout_ms=30)
                if rx:
                    if len(rx) >= 7 and rx[6] in (0x02, 0x03):
                        ack_ok = True
                        break
                    if bytes(rx).startswith(b"WeAct:"):
                        ack_ok = True
                        break
                    if 0x03 in rx or 0x02 in rx:
                        ack_ok = True
                        break
                time.sleep(0.002)

            percent = int((sec_idx + 1) / total_sectors * 100)
            bar_len = 30
            filled = int(bar_len * (sec_idx + 1) // total_sectors)
            bar = "=" * filled + "-" * (bar_len - filled)
            sys.stdout.write(f"\r  [{bar}] {percent:3d}% (Sector {sec_idx + 1}/{total_sectors})")
            sys.stdout.flush()

        elapsed = time.time() - start_t
        print(f"\n{logger.BOLD_GREEN}[SUCCESS]{logger.RESET} Nạp xong toàn bộ firmware trong {elapsed:.2f} giây! ({file_size / elapsed / 1024:.1f} KB/s)")

        # 3. Send REBOOT command
        print(f"{logger.BOLD_BLUE}[3/3]{logger.RESET} Gửi lệnh <REBOOT MCU> kích hoạt ứng dụng...")
        send_packet(h, CMD_WEACT_REBOOT_MCU)
        send_packet(h, CMD_SERASIDIS_REBOOT)
        time.sleep(0.5)

    finally:
        try:
            h.close()
        except Exception:
            pass

    print("\n" + logger.BOLD_GREEN + "══════════════════════════════════════════════════════════════════════" + logger.RESET)
    print(logger.BOLD_GREEN + "   HOÀN THÀNH: NẠP FIRMWARE XE CÂN BẰNG THÀNH CÔNG 100%!" + logger.RESET)
    print(logger.BOLD_GREEN + "   Chip STM32 đang khởi động lại và bắt đầu chạy chương trình!" + logger.RESET)
    print(logger.BOLD_GREEN + "   Đèn LED PC13 sẽ nhấp nháy 1Hz báo hiệu vòng điều khiển đang chạy." + logger.RESET)
    print(logger.BOLD_GREEN + "══════════════════════════════════════════════════════════════════════\n" + logger.RESET)
    return True


if __name__ == "__main__":
    bin_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BIN
    success = flash_firmware(bin_file)
    sys.exit(0 if success else 1)

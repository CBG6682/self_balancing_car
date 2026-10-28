#!/usr/bin/env python3
"""
Real-time UART Bridge Live Tester
Target: ESP32-S3 DevKitC <---> STM32F401 BlackPill
Monitors bidirectional telemetry, tests PING/PONG, and detects STM32 boot banner.
"""

import os
import sys
import time
import serial
import serial.tools.list_ports

# Import shared logger utility
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import logger_utils as logger

BAUD = 115200


def auto_detect_port():
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        return sys.argv[1]
    ports = list(serial.tools.list_ports.comports())
    phys = logger.filter_physical_ports(ports)
    if phys:
        return phys[0].device
    return "COM3" if sys.platform == "win32" else "/dev/ttyACM0"


def main():
    port_name = auto_detect_port()

    print("\n" + "=" * 65)
    print(" KIỂM TRA THÔNG TUYẾN ESP32-S3 <---> STM32 THỜI GIAN THỰC")
    print(f" Cổng: {port_name} @ {BAUD} bps")
    print("=" * 65)
    print(" HƯỚNG DẪN KIỂM TRA:")
    print("  • ESP32 Chân 4 (TX) ---> STM32 Chân A10 (RX)")
    print("  • ESP32 Chân 5 (RX) <--- STM32 Chân A9  (TX)")
    print("  • ESP32 GND        <---> STM32 GND")
    print("  • Mẹo: Bấm nhả nút NRST (Reset) trên STM32 để xem dòng boot ([SYS] MPU6050: OK)")
    print("  • (Bấm Ctrl+C để dừng lại)\n")

    try:
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = BAUD
        ser.timeout = 0.05
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(1.2)
        ser.reset_input_buffer()
    except Exception as e:
        logger.error(f"Không thể mở cổng {port_name}: {e}")
        sys.exit(1)

    logger.info("Đang liên tục gửi 'PING\\n' mỗi 0.5s và lắng nghe phản hồi từ STM32...")
    last_ping = 0
    buf = bytearray()
    ping_count = 0
    pong_count = 0

    try:
        while True:
            now = time.time()
            if now - last_ping > 0.5:
                ser.write(b"PING\n")
                ser.flush()
                last_ping = now
                ping_count += 1

            while ser.in_waiting:
                c = ser.read(1)
                if c == b'\n' or c == b'\r':
                    if len(buf) > 0:
                        msg = buf.decode("latin1", errors="ignore").strip()
                        if msg:
                            if "PONG" in msg:
                                pong_count += 1
                                print(f"\033[92m  ✔ [PHẢN HỒI THÀNH CÔNG TỪ STM32]: {msg} (PONG #{pong_count})\033[0m")
                            elif "[SYS]" in msg or "MPU6050" in msg:
                                print(f"\033[96m  ★ [BOOTLOADER/SYS BANNER]: {msg}\033[0m")
                            elif "STATUS:" in msg or "DATA:" in msg:
                                print(f"\033[93m  ● [TELEMETRY NHẬN ĐƯỢC]: {msg}\033[0m")
                            else:
                                print(f"  >>> [DỮ LIỆU]: {msg}")
                        buf.clear()
                else:
                    buf.extend(c)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n[INFO] Đã dừng kiểm tra.")
        if pong_count > 0:
            logger.success(f"Thông tuyến hoàn hảo! Nhận được {pong_count} phản hồi PONG.")
        else:
            logger.warn(f"Đã gửi {ping_count} gói PING nhưng chưa nhận được PONG.")
            print("  -> Thử đổi chéo 2 đầu dây ở chân 4 và chân 5 trên ESP32.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

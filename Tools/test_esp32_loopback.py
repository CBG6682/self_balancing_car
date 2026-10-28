#!/usr/bin/env python3
"""
ESP32-S3 Loopback Hardware Tester
Tests if GPIO 4 and GPIO 5 on ESP32-S3 DevKitC are properly bridged with a jumper wire.
Verifies ESP32 UART bridge firmware and physical pin functionality.
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
    print(" KIỂM TRA LOOPBACK ESP32-S3 (GPIO 4 <---> GPIO 5)")
    print(f" Cổng: {port_name} @ {BAUD} bps")
    print("=" * 65)
    print(" HƯỚNG DẪN THỰC HIỆN:")
    print("  1. Rút 2 dây đang nối sang STM32 ra.")
    print("  2. Dùng 1 sợi dây nối tắt trực tiếp chân '4' vào chân '5' trên ESP32.")
    print("  3. Chạy script này để kiểm tra xem chuỗi byte gửi đi có dội ngược lại không.")
    print("=" * 65 + "\n")

    try:
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = BAUD
        ser.timeout = 0.2
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(1.2)
        ser.reset_input_buffer()
    except Exception as e:
        logger.error(f"Không thể mở cổng {port_name}: {e}")
        sys.exit(1)

    test_token = f"TEST_LOOPBACK_{int(time.time())}"
    logger.info(f"Đang gửi chuỗi kiểm tra độc lập: '{test_token}' qua chân GPIO 4...")

    received = False
    for attempt in range(1, 6):
        ser.write((test_token + "\n").encode("utf-8"))
        ser.flush()
        time.sleep(0.25)

        while ser.in_waiting:
            resp = ser.readline().decode("utf-8", errors="ignore").strip()
            if test_token in resp:
                print()
                logger.success(f"ESP32 ĐÃ PHẢN HỒI LOOPBACK CHÍNH XÁC: '{resp}' (Lần thử {attempt})")
                print("  ✔ Cổng USB /dev/ttyACM*: HOẠT ĐỘNG TỐT")
                print("  ✔ Firmware esp32_s3_ble_bridge.ino: HOẠT ĐỘNG TỐT")
                print("  ✔ Chân GPIO 4 (TX) & GPIO 5 (RX): HOẠT ĐỘNG HOÀN HẢO 100%!")
                print("\n-> Bạn đã xác định chính xác chân 4 và 5. Giờ có thể cắm 4 vào A10 và 5 vào A9 của STM32.")
                received = True
                break
            elif resp:
                print(f"  [LOG ESP32]: {resp}")

        if received:
            break

    if not received:
        print()
        logger.error("Không nhận lại được chuỗi loopback từ GPIO 5.")
        print("  [CHẨN ĐOÁN LỖI]:")
        print("  1. Bạn cắm nhầm chân (Hãy tìm chân in số '4' và '5' ở hàng bên trái, KHÔNG CẮM chân in chữ TX/RX).")
        print("  2. Sợi dây cắm nối tắt bị đứt ngầm hoặc tiếp xúc lỏng lẻo.")

    ser.close()


if __name__ == "__main__":
    main()

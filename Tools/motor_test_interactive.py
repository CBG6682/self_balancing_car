#!/usr/bin/env python3
"""
Interactive Motor Diagnostic Tester for STM32F401 Self-Balancing Car
Target: TB6612 Dual H-Bridge + GA25 DC Motors via ESP32-S3 UART Bridge.
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


def safe_input(prompt_text, default=""):
    try:
        val = input(prompt_text).strip()
        return val if val else default
    except (EOFError, KeyboardInterrupt):
        return default


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
    print(" BẢNG ĐIỀU KHIỂN KIỂM TRA ĐỘNG CƠ (GA25 + TB6612)")
    print(f" Cổng: {port_name} @ {BAUD} bps")
    print("=" * 65)
    print(" [YÊU CẦU PHẦN CỨNG BẮT BUỘC]:")
    print("  1. Cấp nguồn Pin (7.4V - 11.1V) vào chân VM của TB6612.")
    print("  2. Kê xe lên giá đỡ để bánh xe quay tự do trong không khí.")
    print("  3. Nối STBY của TB6612 vào PB0 (hoặc nối thẳng vào 3.3V).")
    print("=" * 65 + "\n")

    try:
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = BAUD
        ser.timeout = 0.1
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(1.2)
        ser.reset_input_buffer()
    except Exception as e:
        logger.error(f"Không thể mở cổng {port_name}: {e}")
        sys.exit(1)

    def send_cmd(cmd_str):
        ser.write((cmd_str + "\n").encode("utf-8"))
        ser.flush()
        start = time.time()
        got_reply = False
        while time.time() - start < 1.5:
            while ser.in_waiting:
                line = ser.readline().decode("latin1", errors="ignore").strip()
                if line:
                    got_reply = True
                    if "RUN" in line:
                        print(f"\033[92m  ✔ [STM32]: {line}\033[0m")
                    elif "STOP" in line or "DONE" in line:
                        print(f"\033[94m  ⏹ [STM32]: {line}\033[0m")
                    else:
                        print(f"  >>> [STM32]: {line}")
            time.sleep(0.05)
        if not got_reply:
            logger.warn("Không nhận được phản hồi từ STM32. Kiểm tra dây UART nối từ ESP32 (4, 5) sang STM32 (A10, A9)!")

    try:
        while True:
            print("\n" + "─" * 45)
            print("  [1] Motor TRÁI:   Quay TIẾN  (+1500, 0.6s)")
            print("  [2] Motor TRÁI:   Quay LÙI   (-1500, 0.6s)")
            print("  [3] Motor PHẢI:   Quay TIẾN  (+1500, 0.6s)")
            print("  [4] Motor PHẢI:   Quay LÙI   (-1500, 0.6s)")
            print("  [5] CẢ 2 MOTOR:   Quay TIẾN  (+1500, 0.8s)")
            print("  [6] CẢ 2 MOTOR:   Quay LÙI   (-1500, 0.8s)")
            print("  [7] CHU TRÌNH TỰ ĐỘNG 6 PHA (Auto Test)")
            print("  [8] Tùy chỉnh tốc độ PWM (Nhập giá trị)")
            print("  [s] DỪNG KHẨN CẤP (Emergency Stop)")
            print("  [q] Thoát chương trình")
            print("─" * 45)

            choice = safe_input("Nhập lựa chọn (1-8, s, q): ", default="q").lower()

            if choice == "1":
                logger.info("Kích hoạt Motor TRÁI -> TIẾN...")
                send_cmd("TEST:MOTOR_L")
            elif choice == "2":
                logger.info("Kích hoạt Motor TRÁI -> LÙI...")
                send_cmd("TEST:MOTOR_L_REV")
            elif choice == "3":
                logger.info("Kích hoạt Motor PHẢI -> TIẾN...")
                send_cmd("TEST:MOTOR_R")
            elif choice == "4":
                logger.info("Kích hoạt Motor PHẢI -> LÙI...")
                send_cmd("TEST:MOTOR_R_REV")
            elif choice == "5":
                logger.info("Kích hoạt CẢ 2 MOTOR -> TIẾN...")
                send_cmd("TEST:MOTORS")
            elif choice == "6":
                logger.info("Kích hoạt CẢ 2 MOTOR -> LÙI...")
                send_cmd("TEST:MOTORS_REV")
            elif choice == "7":
                logger.info("Bắt đầu chu trình tự động 6 pha...")
                send_cmd("TEST:AUTO")
            elif choice == "8":
                val = safe_input("Nhập PWM Trái và Phải (-4199 đến 4199), ví dụ: 2000,-2000: ", default="")
                if not val:
                    continue
                send_cmd(f"MOTOR:{val}")
            elif choice == "s":
                logger.warn("Dừng khẩn cấp!")
                send_cmd("TEST:STOP")
            elif choice == "q":
                break
            else:
                print("[WARN] Lựa chọn không hợp lệ.")

    except KeyboardInterrupt:
        print("\n[INFO] Đã ngắt bởi người dùng.")
        try:
            ser.write(b"TEST:STOP\n")
        except:
            pass
    finally:
        ser.close()
        logger.info("Cổng Serial đã đóng an toàn.")


if __name__ == "__main__":
    main()

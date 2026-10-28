#!/usr/bin/env python3
"""
Real-time Sensor & Telemetry Live Dashboard
Target: STM32F401 Self-Balancing Robot + MPU6050 (I2C) + GA25 Dual Encoders
Reads live 100Hz stream and displays ASCII tilt bar & encoder counters.
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


def make_ascii_level_bar(angle, max_angle=30.0, bar_width=20):
    """Generates an ASCII level indicator like [    |===o       ]."""
    clamped = max(-max_angle, min(max_angle, angle))
    ratio = (clamped + max_angle) / (2.0 * max_angle)  # 0.0 to 1.0
    pos = int(ratio * (bar_width - 1))
    mid = bar_width // 2

    chars = [" "] * bar_width
    chars[mid] = "|"
    if pos < mid:
        for i in range(pos, mid):
            chars[i] = "="
    elif pos > mid:
        for i in range(mid + 1, pos + 1):
            chars[i] = "="
    chars[pos] = "●"
    return f"[{''.join(chars)}]"


def main():
    port_name = auto_detect_port()

    print("\n" + "=" * 65)
    print(" BẢNG THEO DÕI CẢM BIẾN THỜI GIAN THỰC (MPU6050 + ENCODER)")
    print(f" Cổng: {port_name} @ {BAUD} bps")
    print("=" * 65)
    print(" HƯỚNG DẪN KIỂM TRA:")
    print("  1. Nghiêng xe về trước/sau: Xem thanh đo góc Pitch có đổi chiều không.")
    print("  2. Lấy tay xoay bánh xe TRÁI: Xem số xung Enc_L có tăng/giảm không.")
    print("  3. Lấy tay xoay bánh xe PHẢI: Xem số xung Enc_R có tăng/giảm không.")
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

    # Wake up & enable telemetry stream
    logger.info("Đang bắt tay với STM32 và kích hoạt luồng dữ liệu (START_STREAM)...")
    for _ in range(3):
        ser.write(b"START_STREAM\n")
        ser.flush()
        time.sleep(0.15)

    last_print = time.time()
    packet_count = 0

    try:
        while True:
            line = ser.readline().decode("latin1", errors="ignore").strip()
            if not line:
                # Keep alive stream every 2 seconds
                if time.time() - last_print > 2.0:
                    ser.write(b"START_STREAM\n")
                    ser.flush()
                    last_print = time.time()
                time.sleep(0.01)
                continue

            # Parse DATA:pitch,gyro,pwm,enc_left,enc_right,state
            if line.startswith("DATA:"):
                packet_count += 1
                parts = line[5:].split(",")
                if len(parts) >= 6:
                    try:
                        pitch = float(parts[0])
                        gyro = float(parts[1])
                        pwm = int(parts[2])
                        enc_l = int(parts[3])
                        enc_r = int(parts[4])
                        state_code = int(parts[5])

                        state_str = {0: "IDLE (Nằm yên)", 1: "BALANCING (Cân bằng)", 2: "RECOVERY (Phục hồi)"}.get(state_code, f"STATE_{state_code}")
                        bar = make_ascii_level_bar(pitch, max_angle=25.0, bar_width=24)

                        color = "\033[92m" if abs(pitch) < 3.0 else ("\033[93m" if abs(pitch) < 15.0 else "\033[91m")
                        print(f"\r  {color}Pitch: {pitch:+6.2f}° {bar}\033[0m | Gyro: {gyro:+7.1f}°/s | EncL: {enc_l:8d} | EncR: {enc_r:8d} | {state_str}    ", end="", flush=True)
                    except ValueError:
                        pass
            elif line.startswith("STATUS:") or line.startswith("[SYS]"):
                print(f"\n  [STM32 SYS]: {line}")

    except KeyboardInterrupt:
        print("\n\n[INFO] Đang dừng luồng dữ liệu...")
        try:
            ser.write(b"STOP_STREAM\n")
            ser.flush()
            ser.close()
        except:
            pass
        logger.info(f"Đã dừng. Tổng số gói dữ liệu nhận được: {packet_count}")


if __name__ == "__main__":
    main()

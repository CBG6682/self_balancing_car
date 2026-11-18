#!/usr/bin/env python3
"""
Record high-frequency 100Hz telemetry from STM32 self-balancing robot.
"""
import sys
import time
import csv
import os
import serial

PORT = "/dev/ttyACM0"
BAUD = 115200
DURATION_SEC = 10.0
CSV_OUT = "Tools/real_telemetry.csv"
BACKUP_CSV = "Tools/real_telemetry_backward_run.csv"

def main():
    print(f"\n==================================================")
    print(f"  BẮT ĐẦU THU THẬP DỮ LIỆU TELEMETRY THỰC TẾ (100Hz)")
    print(f"  Thời gian ghi: {DURATION_SEC} giây")
    print(f"  Cổng: {PORT}")
    print(f"==================================================\n")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except Exception as e:
        print(f"[ERR] Không thể mở cổng {PORT}: {e}")
        sys.exit(1)

    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Query current PID settings
    ser.write(b"GET_PID\n")
    time.sleep(0.1)
    pid_resp = ser.read_all().decode("utf-8", errors="ignore").strip()
    print(f"[THÔNG SỐ HIỆN TẠI] {pid_resp}")

    # Switch to AUTO mode
    print("[SYS] Gửi lệnh AUTO...")
    ser.write(b"AUTO\n")
    time.sleep(0.1)
    auto_resp = ser.read_all().decode("utf-8", errors="ignore").strip()
    print(f"[RESP AUTO] {auto_resp}")

    # Start stream
    print("[SYS] Kích hoạt dòng dữ liệu START_STREAM...")
    ser.write(b"START_STREAM\n")
    time.sleep(0.05)

    records = []
    start_t = time.time()
    last_print_t = 0

    print("\nĐang ghi dữ liệu (Bạn hãy đặt xe cân bằng tự nhiên, nếu ngã hãy dựng lại)...")
    print("-" * 65)

    try:
        while (time.time() - start_t) < DURATION_SEC:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("DATA:"):
                parts = line[5:].split(",")
                if len(parts) >= 6:
                    t_sample = time.time() - start_t
                    record = {
                        "time": f"{t_sample:.3f}",
                        "pitch_angle": parts[0],
                        "gyro_rate": parts[1],
                        "pwm_command": parts[2],
                        "enc_left": parts[3],
                        "enc_right": parts[4],
                        "fsm_state": parts[5]
                    }
                    records.append(record)

                    # Print status every ~0.15s
                    if (t_sample - last_print_t) >= 0.15:
                        last_print_t = t_sample
                        p = float(parts[0])
                        g = float(parts[1])
                        pwm = int(parts[2])
                        el = int(parts[3])
                        er = int(parts[4])
                        st = parts[5]
                        bar = "#" * int(t_sample / DURATION_SEC * 20)
                        print(f"\r[{bar:<20}] {t_sample:4.1f}s | PITCH: {p:+6.2f}° | GYRO: {g:+6.1f} | PWM: {pwm:+5d} | ENC: ({el:+5d}, {er:+5d}) | ST: {st}", end="", flush=True)

    finally:
        ser.write(b"STOP_STREAM\n")
        time.sleep(0.1)
        ser.close()

    print(f"\n" + "-" * 65)
    print(f"\n[HOÀN TẤT] Đã thu thập {len(records)} mẫu trong {DURATION_SEC:.1f} giây!")

    if len(records) > 0:
        # Save to main CSV
        with open(CSV_OUT, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["time", "pitch_angle", "gyro_rate", "pwm_command", "enc_left", "enc_right", "fsm_state"])
            writer.writeheader()
            writer.writerows(records)

        # Save backup
        with open(BACKUP_CSV, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["time", "pitch_angle", "gyro_rate", "pwm_command", "enc_left", "enc_right", "fsm_state"])
            writer.writeheader()
            writer.writerows(records)

        print(f"[LƯU TRỮ] Đã lưu dữ liệu vào {CSV_OUT} và {BACKUP_CSV}\n")
    else:
        print("[ERR] Không nhận được mẫu dữ liệu nào. Vui lòng kiểm tra cáp UART.")

if __name__ == "__main__":
    main()

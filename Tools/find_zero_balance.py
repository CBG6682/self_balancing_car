#!/usr/bin/env python3
"""
Live Pitch Visualizer to find exact mechanical zero balance point.
"""
import serial
import time
import sys

def main():
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    print("\n=======================================================")
    print("  TÌM ĐIỂM TRỌNG TÂM CÂN BẰNG TỰ NHIÊN (ZERO BALANCE)")
    print("  • Hãy để dây cáp USB chùng xuống sàn (không kéo căng xe)")
    print("  • Dùng 1 đầu ngón tay giữ xe ở vị trí cảm thấy nhẹ nhất")
    print("  • Xem góc PITCH hiển thị bên dưới:")
    print("=======================================================\n")

    try:
        for _ in range(50):
            ser.write(b"STATUS\n")
            time.sleep(0.1)
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("STATUS:"):
                # Extract PITCH
                parts = dict(kv.split("=") for kv in line[7:].split(",") if "=" in kv)
                pitch = parts.get("PITCH", "0.0")
                gyro = parts.get("GYRO", "0.0")
                state = parts.get("STATE", "")
                p_val = float(pitch)
                bar_len = int(abs(p_val) * 3)
                if p_val > 0:
                    vis = f"[{' ':>15}|{'#'*min(bar_len, 15):<15}]"
                else:
                    vis = f"[{'#'*min(bar_len, 15):>15}|{' ':15}]"
                sys.stdout.write(f"\r  Góc PITCH: {p_val:+6.2f}°  {vis}  Gyro: {gyro}°/s  Trạng thái: {state}   ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\n\nHoàn tất đọc dữ liệu.")

if __name__ == "__main__":
    main()

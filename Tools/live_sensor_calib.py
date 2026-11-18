#!/usr/bin/env python3
"""
==============================================================================
Live Sensor Reader & Calibration Freezing Tool for STM32 Self-Balancing Robot
Target: STM32F401CCU6 via ESP32-S3 BLE Bridge (/dev/ttyACM0)

Features:
1. Continuously displays real-time IMU readings (Pitch, Gyro rate, stability).
2. Pressing ENTER instantly locks / freezes the exact sensor values at that moment.
3. Automatically calculates calibration offsets and allows instant one-click 
   injection into STM32 RAM or permanent update to C header file.
==============================================================================
"""

import sys
import os
import time
import math
import serial
import glob
import threading
from collections import deque

DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200

def find_serial_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if ports:
        return ports[0]
    return DEFAULT_PORT

# ANSI Color Codes
CYAN = "\033[96m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"
CLEAR_LINE = "\033[2K\r"

class SensorMonitor:
    def __init__(self, port=None, baud=BAUD):
        self.port = port if port else find_serial_port()
        self.baud = baud
        self.ser = None
        self.running = False
        self.frozen = False
        self.latest_data = {
            "pitch": 0.0,
            "gyro": 0.0,
            "pwm": 0,
            "enc_l": 0,
            "enc_r": 0,
            "state": 0,
            "time": 0.0
        }
        self.history = deque(maxlen=50) # 0.5s buffer at 100Hz
        self.lock = threading.Lock()
        self.current_ram_offset = 0.0
        self.current_ram_pid = ""

    def connect(self):
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baud
            self.ser.timeout = 0.2
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            time.sleep(0.1)
            # Stop any pending stream
            self.ser.write(b"STOP_STREAM\n")
            time.sleep(0.05)
            self.ser.reset_input_buffer()

            # Query current PID and offset
            self.ser.write(b"GET_PID\n")
            time.sleep(0.05)
            resp = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if "OFFSET:" in resp:
                parts = resp.split("OFFSET:")
                self.current_ram_offset = float(parts[1])
                self.current_ram_pid = parts[0]
            return True
        except Exception as e:
            print(f"{RED}[LỖI] Không thể mở cổng {self.port}: {e}{RESET}")
            return False

    def send_cmd(self, cmd_str):
        if not self.ser:
            return ""
        self.ser.write((cmd_str.strip() + "\n").encode())
        time.sleep(0.05)
        resp = self.ser.readline().decode("utf-8", errors="ignore").strip()
        return resp

    def stream_thread_func(self):
        self.ser.write(b"START_STREAM\n")
        time.sleep(0.05)
        last_display_time = 0
        last_rx_time = time.time()
        last_warning_time = 0

        while self.running:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            now = time.time()
            if not line or not line.startswith("DATA:"):
                if not self.frozen and (now - last_rx_time) > 1.5 and (now - last_warning_time) > 1.0:
                    last_warning_time = now
                    sys.stdout.write(f"{CLEAR_LINE}{YELLOW}[ĐANG CHỜ TÍN HIỆU]{RESET} Chưa nhận được dữ liệu từ STM32... Hãy kiểm tra công tắc nguồn pin của xe hoặc bấm nhả nút Reset (NRST)!")
                    sys.stdout.flush()
                continue

            last_rx_time = now

            parts = line[5:].split(",")
            if len(parts) < 6:
                continue

            try:
                pitch = float(parts[0])
                gyro = float(parts[1])
                pwm = int(parts[2])
                el = int(parts[3])
                er = int(parts[4])
                state = int(parts[5])

                with self.lock:
                    self.latest_data = {
                        "pitch": pitch,
                        "gyro": gyro,
                        "pwm": pwm,
                        "enc_l": el,
                        "enc_r": er,
                        "state": state,
                        "time": time.time()
                    }
                    self.history.append((pitch, gyro))

                # Display dynamic gauge every 50ms if not frozen
                now = time.time()
                if not self.frozen and (now - last_display_time) >= 0.05:
                    last_display_time = now
                    self.render_live_line(pitch, gyro, state)

            except ValueError:
                continue

    def render_live_line(self, pitch, gyro, state):
        # Draw ASCII balance gauge [-15° ... 0° ... +15°]
        bar_w = 15
        clamped_p = max(-15.0, min(15.0, pitch))
        pos = int((clamped_p + 15.0) / 30.0 * (bar_w * 2))
        gauge = list("·" * (bar_w * 2 + 1))
        gauge[bar_w] = "|"
        if 0 <= pos < len(gauge):
            gauge[pos] = "O"
        gauge_str = "".join(gauge)

        # Calculate stability (standard deviation over last 0.5s)
        std_pitch = 0.0
        with self.lock:
            if len(self.history) >= 10:
                pitches = [p for p, g in self.history]
                mean_p = sum(pitches) / len(pitches)
                std_pitch = math.sqrt(sum((x - mean_p)**2 for x in pitches) / len(pitches))

        stab_color = GREEN if std_pitch < 0.20 else (YELLOW if std_pitch < 0.80 else RED)
        state_str = f"{GREEN}BALANCING{RESET}" if state == 1 else f"{YELLOW}IDLE/STANDBY{RESET}"

        display = (f"{CLEAR_LINE}{CYAN}[{gauge_str}]{RESET} "
                   f"Góc Pitch: {BOLD}{pitch:+6.2f}°{RESET} | "
                   f"Gyro: {gyro:+6.1f}°/s | "
                   f"Độ rung: {stab_color}±{std_pitch:.2f}°{RESET} | "
                   f"{state_str} ")
        sys.stdout.write(display)
        sys.stdout.flush()

    def start(self):
        self.running = True
        self.t = threading.Thread(target=self.stream_thread_func, daemon=True)
        self.t.start()

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.write(b"STOP_STREAM\n")
                time.sleep(0.05)
                self.ser.close()
            except:
                pass

def apply_offset_to_c_header(offset_deg):
    header_path = "Modules/Inc/pid_AutoConfig.h"
    if not os.path.exists(header_path):
        return False
    try:
        with open(header_path, "r") as f:
            content = f.read()
        import re
        pattern = r"(#define\s+MECHANICAL_BALANCE_OFFSET_DEG\s+)\([^\)]+\)"
        new_val = f"\\g<1>(+{offset_deg:.2f}f)" if offset_deg >= 0 else f"\\g<1>({offset_deg:.2f}f)"
        new_content = re.sub(pattern, new_val, content)
        with open(header_path, "w") as f:
            f.write(new_content)
        return True
    except Exception as e:
        print(f"[ERR] Không thể cập nhật {header_path}: {e}")
        return False

def main():
    print("\n" + "═" * 70)
    print(f" {BOLD}CÔNG CỤ ĐỌC CẢM BIẾN THỜI GIAN THỰC & KHOÁ GÓC CALIB TỰ ĐỘNG{RESET} ")
    print(" Target: STM32F401CCU6 qua ESP32-S3 BLE Bridge (/dev/ttyACM0)")
    print("═" * 70)
    print(" HƯỚNG DẪN:")
    print(" 1. Giữ xe đứng thẳng thăng bằng tự nhiên bằng tay trên mặt phẳng.")
    print(f" 2. Khi xe ổn định, {BOLD}{GREEN}NHẤN PHÍM ENTER{RESET} để KHOÁ giá trị ngay thời điểm đó.")
    print(" 3. Sau khi khoá, bạn có thể chọn nạp Offset trực tiếp vào RAM hoặc lưu lại.")
    print("═" * 70 + "\n")

    monitor = SensorMonitor()
    if not monitor.connect():
        sys.exit(1)

    print(f"  [RAM HIỆN TẠI] Offset đang lưu: {BOLD}{monitor.current_ram_offset:+.2f}°{RESET}")
    print("  Đang khởi động luồng đọc cảm biến tốc độ cao (100Hz)...\n")
    monitor.start()

    try:
        while True:
            # Wait for user to press ENTER to freeze
            try:
                input()
            except (KeyboardInterrupt, EOFError):
                break

            # ENTER pressed -> Freeze!
            monitor.frozen = True
            time.sleep(0.02) # Allow last sample to settle

            with monitor.lock:
                frozen = dict(monitor.latest_data)
                hist = list(monitor.history)

            # Compute statistics
            if hist:
                pitches = [p for p, g in hist]
                mean_p = sum(pitches) / len(pitches)
                std_p = math.sqrt(sum((x - mean_p)**2 for x in pitches) / len(pitches))
                gyros = [g for p, g in hist]
                mean_g = sum(gyros) / len(gyros)
            else:
                mean_p = frozen["pitch"]
                std_p = 0.0
                mean_g = frozen["gyro"]

            # Display frozen summary card
            print("\n\n" + "╔" + "═" * 66 + "╗")
            print(f"║ {BOLD}{YELLOW}          ĐÃ KHOÁ GIÁ TRỊ CẢM BIẾN TẠI THỜI ĐIỂM BẤM ENTER        {RESET} ║")
            print("╠" + "═" * 66 + "╣")
            print(f"║  • {BOLD}Góc Pitch tức thời{RESET}     : {CYAN}{frozen['pitch']:+7.2f}°{RESET}                             ║")
            print(f"║  • {BOLD}Góc Pitch trung bình (0.5s){RESET}: {GREEN}{mean_p:+7.2f}°{RESET}                             ║")
            print(f"║  • {BOLD}Vận tốc góc Gyro Rate{RESET}  : {YELLOW}{mean_g:+7.2f}°/s{RESET}                            ║")
            print(f"║  • {BOLD}Độ rung nhiễu (StdDev){RESET} : ±{std_p:5.2f}° ({'Rất vững' if std_p < 0.25 else 'Hơi rung'})                  ║")
            print(f"║  • {BOLD}Offset hiện tại trong RAM{RESET}: {monitor.current_ram_offset:+7.2f}°                             ║")
            print("╠" + "═" * 66 + "╣")
            print(f"║  ★ {BOLD}GIÁ TRỊ CALIB ĐỀ XUẤT{RESET}   : {BOLD}{GREEN}SET_OFFSET:{mean_p:+.2f}{RESET}                       ║")
            print("╚" + "═" * 66 + "╝\n")

            # Selection menu
            print("  Chọn thao tác:")
            print(f"    [{BOLD}1{RESET} hoặc {BOLD}Y{RESET}]  -> {GREEN}NẠP NGAY{RESET} Offset = {mean_p:+.2f}° vào RAM STM32 qua ESP32")
            print(f"    [{BOLD}2{RESET}]       -> Nạp vào RAM {BOLD}VÀ{RESET} cập nhật file mã nguồn pid_AutoConfig.h")
            print(f"    [{BOLD}3{RESET} / {BOLD}ENTER{RESET}] -> Tiếp tục đọc cảm biến (Mở khoá)")
            print(f"    [{BOLD}Q{RESET}]       -> Thoát chương trình\n")

            choice = input("  Nhập lựa chọn của bạn: ").strip().lower()

            if choice in ["1", "y", "yes"]:
                # Send SET_OFFSET to STM32 RAM
                monitor.ser.write(b"STOP_STREAM\n")
                time.sleep(0.05)
                monitor.ser.reset_input_buffer()
                resp = monitor.send_cmd(f"SET_OFFSET:{mean_p:.2f}")
                print(f"\n  {GREEN}[THÀNH CÔNG]{RESET} Phản hồi STM32: {resp}")
                monitor.current_ram_offset = mean_p
                # Resume streaming
                monitor.ser.write(b"START_STREAM\n")
                time.sleep(0.05)
                print("  -> Đã nạp thành công! Tiếp tục đọc cảm biến...")

            elif choice == "2":
                # Update RAM + C header
                monitor.ser.write(b"STOP_STREAM\n")
                time.sleep(0.05)
                monitor.ser.reset_input_buffer()
                resp = monitor.send_cmd(f"SET_OFFSET:{mean_p:.2f}")
                print(f"\n  {GREEN}[RAM THÀNH CÔNG]{RESET} Phản hồi STM32: {resp}")
                monitor.current_ram_offset = mean_p
                if apply_offset_to_c_header(mean_p):
                    print(f"  {GREEN}[FILE C THÀNH CÔNG]{RESET} Đã ghi MECHANICAL_BALANCE_OFFSET_DEG (+{mean_p:.2f}f) vào Modules/Inc/pid_AutoConfig.h")
                # Resume streaming
                monitor.ser.write(b"START_STREAM\n")
                time.sleep(0.05)
                print("  -> Tiếp tục đọc cảm biến...")

            elif choice == "q":
                print("\n  Đang thoát chương trình...")
                break
            else:
                print("\n  -> Tiếp tục đọc cảm biến...")

            # Unfreeze and loop
            monitor.frozen = False
            time.sleep(0.1)

    finally:
        monitor.stop()
        print(f"\n{GREEN}[HOÀN TẤT]{RESET} Đã dừng luồng telemetry an toàn.\n")

if __name__ == "__main__":
    main()

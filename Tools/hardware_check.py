#!/usr/bin/env python3
"""
Hardware Bringup, Interactive Diagnostics & Live Tuning Suite
Target: STM32F401CCU6 Self-Balancing Robot + TB6612 + MPU6050 + ESP32-S3

Features:
1. Auto-detection & Handshake over USB-UART / ESP32-S3 CDC.
2. Live IMU Pitch angle visualizer with graphical level meter.
3. Live Encoder pulse direction verification.
4. Safe motor test pulse (direction & deadzone verification).
5. Hot motor/encoder polarity inversion without recompilation.
6. One-click zero balance trim calibration.
7. Real-time telemetry streaming & automated PID tuning on real hardware.
8. Built-in Simulation Mode for testing without physical wires.
"""

import os
import sys
import time
import math
import select
import yaml
import numpy as np
import serial
import serial.tools.list_ports

# Import shared logger utility
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import logger_utils as logger

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "robot_config.yaml")
REAL_CSV_FILE = os.path.join(SCRIPT_DIR, "real_telemetry.csv")


def load_config():
    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def save_config(cfg):
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        yaml.dump(cfg, f, allow_unicode=True, sort_keys=False)


class MockSerial:
    """Emulates STM32F401 serial interface for testing without hardware."""
    def __init__(self):
        self.is_streaming = False
        self.t_start = time.time()
        self.pitch = 0.0
        self.gyro = 0.0
        self.pwm = 0
        self.enc_l = 0
        self.enc_r = 0
        self.zero_offset = 0.0
        self.state = "BALANCING"
        self._rx_queue = []

    def write(self, data: bytes):
        cmd = data.decode("utf-8", errors="ignore").strip()
        if cmd in ("PING", "ESP:PING"):
            self._rx_queue.append(b"PONG\n")
        elif cmd == "STATUS":
            self._rx_queue.append(
                f"STATUS:STATE={self.state},PITCH={self.pitch:.2f},GYRO={self.gyro:.2f},SPD=0.00,ENC_L={self.enc_l},ENC_R={self.enc_r},MPU=1,BLE=1\n".encode("utf-8")
            )
        elif cmd == "START_STREAM":
            self.is_streaming = True
            self.t_start = time.time()
            self._rx_queue.append(b"STREAM_STARTED\n")
        elif cmd == "STOP_STREAM":
            self.is_streaming = False
            self._rx_queue.append(b"STREAM_STOPPED\n")
        elif cmd == "CALIB_ZERO":
            self.zero_offset = self.pitch
            self._rx_queue.append(b"OK:ZERO_SET\n")
        elif cmd.startswith("TEST:MOTOR"):
            self._rx_queue.append(b"TEST:MOTORS_RUN\n")
        elif cmd.startswith("SET_POLARITY"):
            self._rx_queue.append(b"OK:POLARITY_SAVED\n")

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._rx_queue.clear()

    def reset_output_buffer(self):
        pass

    def readline(self) -> bytes:
        if self._rx_queue:
            return self._rx_queue.pop(0)

        if self.is_streaming:
            time.sleep(0.02)
            t = time.time() - self.t_start
            # 2-wheeled inverted pendulum physics simulation
            self.pitch = 3.5 * math.sin(2.8 * t) * math.exp(-0.06 * (t % 12)) + float(np.random.normal(0, 0.12)) - self.zero_offset
            self.gyro = 9.8 * math.cos(2.8 * t) + float(np.random.normal(0, 0.3))
            self.pwm = int(min(999, max(-999, 48.0 * self.pitch + 2.8 * self.gyro)))
            self.enc_l += int(self.pwm * 0.015 + np.random.normal(0, 0.5))
            self.enc_r += int(self.pwm * 0.015 + np.random.normal(0, 0.5))
            self.state = "BALANCING" if abs(self.pitch) < 18.0 else "FALLEN"
            return f"DATA:{self.pitch:.2f},{self.gyro:.2f},{self.pwm},{self.enc_l},{self.enc_r},{self.state}\n".encode("utf-8")
        else:
            time.sleep(0.05)
            return b""

    def close(self):
        self.is_streaming = False


def find_stm32_serial():
    """Scans for available serial ports and attempts PING/PONG handshake."""
    config = load_config()
    baudrate = int(config["uart"].get("baudrate", 115200))
    all_ports = serial.tools.list_ports.comports()
    physical_ports = logger.filter_physical_ports(all_ports)

    if not physical_ports:
        return None, None

    print(f"{logger.BOLD_BLUE}[SCAN]{logger.RESET} Tìm thấy {len(physical_ports)} cổng phần cứng khả dụng:")
    for p in physical_ports:
        print(f"  • {p.device}: {p.description}")

    for port_info in physical_ports:
        port_name = port_info.device
        print(f"\r  -> Đang thử bắt tay PING/PONG trên {port_name}...", end="", flush=True)
        try:
            ser = serial.Serial()
            ser.port = port_name
            ser.baudrate = baudrate
            ser.timeout = 0.8
            ser.dtr = False
            ser.rts = False
            ser.open()

            # Allow ESP32 / USB-UART bridge to settle and flush any boot banner
            time.sleep(1.2)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # Test handshake with retries
            for attempt in range(4):
                ser.write(b"PING\nSTATUS\n")
                ser.flush()
                for _ in range(6):
                    reply = ser.readline().decode("utf-8", errors="ignore").strip()
                    if "PONG" in reply or reply.startswith("STATUS:") or "STM32F401" in reply or reply.startswith("DATA:"):
                        print()
                        logger.success(f"Kết nối thành công với STM32 trên cổng {port_name}!")
                        return ser, port_name
                    time.sleep(0.05)
                time.sleep(0.3)
            ser.close()
        except (serial.SerialException, OSError):
            continue

    print()
    return None, None


def connect_or_wait_serial():
    """Connects to real STM32 serial port or enters live hotplug listener / simulator."""
    ser, port = find_stm32_serial()
    if ser is not None:
        return ser, port

    logger.warn("Chưa phát hiện cổng Serial vật lý nào (/dev/ttyUSB*, /dev/ttyACM*).")

    print(f"\n{logger.BOLD_CYAN}══════════════════════════════════════════════════════════════════════{logger.RESET}")
    print(f"{logger.BOLD_WHITE}  TẠI SAO ĐÃ CẮM CÁP MÀ BƯỚC NÀY VẪN BÁO 'CHƯA NHẬN'?{logger.RESET}")
    print(f"{logger.BOLD_CYAN}══════════════════════════════════════════════════════════════════════{logger.RESET}")
    print("  • Cáp USB Type-C trên mạch STM32 BlackPill CHỈ dùng cho WeAct HID Bootloader (Bước nạp).")
    print("  • Ở bước này (Telemetry & Tinh chỉnh PID), STM32 truyền dữ liệu qua chân USART1:")
    print(f"    - Chân {logger.BOLD_CYAN}PA9{logger.RESET}  : USART1_TX (Chân truyền dữ liệu từ xe ra ngoài)")
    print(f"    - Chân {logger.BOLD_CYAN}PA10{logger.RESET} : USART1_RX (Chân nhận lệnh từ máy tính vào xe)")
    print(f"    - Chân {logger.BOLD_CYAN}GND{logger.RESET}  : Nối chung mass (Ground)\n")

    print(f"  • {logger.BOLD_YELLOW}Sơ đồ nối dây với mạch USB-UART (CH340/CP2102) hoặc ESP32-S3:{logger.RESET}")
    print(f"    ┌────────────────────────┐          ┌────────────────────────┐")
    print(f"    │ Mạch USB-UART / ESP32  │          │ STM32F401 BlackPill    │")
    print(f"    │   Chân TX (Truyền)     │─────────>│   Chân PA10 (Nhận RX)  │")
    print(f"    │   Chân RX (Nhận)       │<─────────│   Chân PA9  (Truyền TX)│")
    print(f"    │   Chân GND (Mass)      │<────────>│   Chân GND             │")
    print(f"    │   Cổng USB             │─────────>│   Cắm vào Laptop       │")
    print(f"    └────────────────────────┘          └────────────────────────┘")
    print(f"    (Lưu ý chéo chân: TX mạch này nối vào RX mạch kia, GND nối GND)")
    print(f"{logger.BOLD_CYAN}══════════════════════════════════════════════════════════════════════\n{logger.RESET}")

    print(f"{logger.BOLD_BLUE}>>> Đang ở chế độ LẮNG NGHE THỜI GIAN THỰC (LIVE HOTPLUG LISTENER)...{logger.RESET}")
    print("  • Cắm mạch USB-UART / ESP32-S3 vào bất kỳ cổng USB nào của laptop...")
    print(f"  • Hoặc gõ {logger.BOLD_GREEN}'s'{logger.RESET} + Enter để chạy ngay chế độ {logger.BOLD_GREEN}MÔ PHỎNG (Simulator){logger.RESET} kiểm thử.")
    print("  • (Bấm Ctrl+C bất cứ lúc nào để dừng lại)\n")

    spinner = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
    idx = 0

    try:
        while True:
            # Check for user keyboard input without blocking
            if select.select([sys.stdin], [], [], 0.0)[0]:
                user_line = sys.stdin.readline().strip().lower()
                if user_line in ("s", "sim", "mock"):
                    print()
                    logger.info("Chuyển sang chế độ MÔ PHỎNG (VIRTUAL SIMULATOR)...")
                    return MockSerial(), "MÔ PHỎNG (VIRTUAL COM)"

            # Scan physical serial ports
            all_ports = serial.tools.list_ports.comports()
            phys = logger.filter_physical_ports(all_ports)
            if phys:
                print()
                logger.success(f"Phát hiện cổng Serial phần cứng mới: {phys[0].device} ({phys[0].description})!")
                time.sleep(0.5)
                ser, p = find_stm32_serial()
                if ser is not None:
                    return ser, p
                else:
                    logger.warn(f"Cổng {phys[0].device} đã kết nối nhưng chưa nhận được phản hồi PONG từ STM32.")
                    print("  -> Kiểm tra: Đã cấp nguồn cho STM32 chưa? Chân TX/RX đã cắm chéo đúng chưa?")

            char = spinner[idx % len(spinner)]
            idx += 1
            sys.stdout.write(f"\r  {char} Đang theo dõi cổng USB... (Cắm USB-UART hoặc gõ 's' để chạy mô phỏng) ")
            sys.stdout.flush()
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n")
        logger.info("Người dùng hủy bỏ theo dõi.")
        return None, None


def render_ascii_level(angle_deg, max_range=25.0, width=30):
    """Renders a graphical level meter for tilt angle: [-----|=====]"""
    norm = max(-1.0, min(1.0, angle_deg / max_range))
    center = width // 2
    bar = ["-"] * width
    bar[center] = "|"

    pos = int(center + norm * (center - 1))
    pos = max(0, min(width - 1, pos))

    if pos > center:
        for i in range(center + 1, pos + 1):
            bar[i] = "="
    elif pos < center:
        for i in range(pos, center):
            bar[i] = "="

    bar_str = "".join(bar)
    color = logger.BOLD_GREEN if abs(angle_deg) < 3.0 else (logger.BOLD_YELLOW if abs(angle_deg) < 15.0 else logger.BOLD_RED)
    return f"{color}[{bar_str}]{logger.RESET}"


def mode_live_pitch(ser):
    """Live pitch stream & IMU level meter."""
    logger.sub_header("THEO DÕI GÓC NGHIÊNG THỜI GIAN THỰC (LIVE IMU PITCH)")
    print("  • Hướng dẫn kiểm tra:")
    print("    - Nghiêng xe TỚI TRƯỚC : Góc Pitch phải TĂNG (+)")
    print("    - Nghiêng xe VỀ SAU    : Góc Pitch phải GIẢM (-)")
    print("  (Nhấn Ctrl+C để quay lại Menu chính)\n")

    ser.write(b"START_STREAM\n")
    ser.flush()

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("DATA:"):
                # DATA:<pitch>,<gyro>,<pwm>,<enc_l>,<enc_r>,<state>
                parts = line[5:].split(",")
                if len(parts) >= 6:
                    pitch = float(parts[0])
                    gyro = float(parts[1])
                    state = parts[5]
                    meter = render_ascii_level(pitch)
                    sys.stdout.write(f"\r  {meter}  Pitch: {pitch:+6.2f}° | Gyro: {gyro:+7.2f}°/s | Trạng thái: {state:10s}")
                    sys.stdout.flush()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        ser.write(b"STOP_STREAM\n")
        ser.flush()


def mode_live_encoders(ser):
    """Live encoder pulse counters."""
    logger.sub_header("KIỂM TRA ĐỌC ENCODER 2 BÁNH XE (LIVE ENCODER COUNTERS)")
    print("  • Hướng dẫn kiểm tra:")
    print("    - Dùng tay lăn xe TIẾN THẲNG tới trước:")
    print("    - Cả 2 số xung Trái và Phải đều phải TĂNG (+).")
    print("    - Nếu bánh nào giảm (-), bánh đó đang bị ngược pha encoder!")
    print("  (Nhấn Ctrl+C để quay lại Menu chính)\n")

    ser.write(b"START_STREAM\n")
    ser.flush()

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("DATA:"):
                parts = line[5:].split(",")
                if len(parts) >= 5:
                    enc_l = int(parts[3])
                    enc_r = int(parts[4])
                    pwm = int(parts[2])
                    sys.stdout.write(f"\r  • Encoder TRÁI: {enc_l:+8d} xung | Encoder PHẢI: {enc_r:+8d} xung | PWM: {pwm:+5d}")
                    sys.stdout.flush()
            time.sleep(0.03)
    except KeyboardInterrupt:
        print()
    finally:
        ser.write(b"STOP_STREAM\n")
        ser.flush()


def mode_test_motors(ser):
    """Test motor directions via short pulses."""
    logger.sub_header("THỬ NGHIỆM CHIỀU QUAY ĐỘNG CƠ TB6612")
    print("  [CẢNH BÁO] Hãy nhấc bổng xe lên khỏi mặt bàn hoặc đặt trên giá đỡ kê trục bánh!\n")
    input("  Nhấn [ENTER] để bắt đầu thử quay ĐỘNG CƠ TRÁI tiến 0.5s...")
    ser.write(b"TEST:MOTOR_L\n")
    ser.flush()
    time.sleep(0.8)
    ans_l = input("  -> Bánh TRÁI có quay TIẾN TỚI trước không? (y/n) [y]: ").strip().lower()

    input("\n  Nhấn [ENTER] để bắt đầu thử quay ĐỘNG CƠ PHẢI tiến 0.5s...")
    ser.write(b"TEST:MOTOR_R\n")
    ser.flush()
    time.sleep(0.8)
    ans_r = input("  -> Bánh PHẢI có quay TIẾN TỚI trước không? (y/n) [y]: ").strip().lower()

    cfg = load_config()
    changed = False

    if ans_l == "n":
        curr = int(cfg["motor"].get("motor_left_polarity", 1))
        cfg["motor"]["motor_left_polarity"] = -curr
        changed = True
        logger.warn(f"Đã đảo chiều động cơ TRÁI sang {cfg['motor']['motor_left_polarity']}")

    if ans_r == "n":
        curr = int(cfg["motor"].get("motor_right_polarity", -1))
        cfg["motor"]["motor_right_polarity"] = -curr
        changed = True
        logger.warn(f"Đã đảo chiều động cơ PHẢI sang {cfg['motor']['motor_right_polarity']}")

    if changed:
        save_config(cfg)
        # Send update to STM32 RAM
        ml = cfg["motor"]["motor_left_polarity"]
        mr = cfg["motor"]["motor_right_polarity"]
        el = cfg["motor"].get("encoder_left_polarity", 1)
        er = cfg["motor"].get("encoder_right_polarity", -1)
        cmd = f"SET_POLARITY:{ml},{mr},{el},{er}\n".encode("utf-8")
        ser.write(cmd)
        ser.flush()
        logger.success("Đã nạp chiều quay mới vào STM32 và lưu vào robot_config.yaml!")
    else:
        logger.success("Chiều quay của cả 2 động cơ đều đã CHUẨN XÁC!")


def mode_calibrate_zero(ser):
    """Calibrate mechanical balance zero angle."""
    logger.sub_header("CĂN CHỈNH GÓC CÂN BẰNG TRỌNG TÂM CƠ HỌC (ZERO BALANCE OFFSET)")
    print("  • Hướng dẫn:")
    print("    1. Đặt xe trên mặt sàn phẳng.")
    print("    2. Dùng 2 ngón tay giữ nhẹ xe ở điểm cân bằng tự nhiên nhất (không bị ngã về trước hay sau).")
    print("    3. Giữ xe đứng yên hoàn toàn và nhấn ENTER để chốt điểm 0.0°.\n")
    input("  Nhấn [ENTER] để ghi nhận góc cân bằng...")

    ser.write(b"CALIB_ZERO\n")
    ser.flush()
    time.sleep(0.3)
    reply = ser.readline().decode("utf-8", errors="ignore").strip()
    logger.success(f"STM32 phản hồi: {reply}")


def mode_collect_telemetry_and_autotune(ser):
    """Collect 8s live telemetry and trigger Nelder-Mead auto-tuning."""
    logger.sub_header("THU THẬP DỮ LIỆU THỰC TẾ & TỰ ĐỘNG TINH CHỈNH PID (AUTO-TUNING)")
    duration_sec = 8.0
    print(f"  • Chuẩn bị: Đặt xe thăng bằng trên sàn và giữ nhẹ.")
    print(f"  • Hệ thống sẽ thu thập dữ liệu trong {duration_sec}s...")
    input("  Nhấn [ENTER] để bắt đầu thu thập dữ liệu...")

    ser.reset_input_buffer()
    ser.write(b"START_STREAM\n")
    ser.flush()

    start_t = time.time()
    records = []
    print("  Đang ghi nhận dữ liệu từ cảm biến STM32: [", end="", flush=True)

    try:
        while (time.time() - start_t) < duration_sec:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if raw.startswith("DATA:"):
                parts = raw[5:].split(",")
                if len(parts) >= 6:
                    t_now = time.time() - start_t
                    records.append({
                        "time": f"{t_now:.3f}",
                        "pitch_angle": parts[0],
                        "gyro_rate": parts[1],
                        "pwm_command": parts[2],
                        "enc_left": parts[3],
                        "enc_right": parts[4],
                        "fsm_state": parts[5]
                    })
                    if len(records) % 40 == 0:
                        print("=", end="", flush=True)
    finally:
        ser.write(b"STOP_STREAM\n")
        ser.flush()

    print("] HOÀN TẤT!")

    if len(records) < 80:
        logger.error(f"Thu thập không đủ mẫu (chỉ nhận {len(records)} mẫu). Vui lòng thử lại.")
        return

    # Write to real_telemetry.csv
    import csv
    with open(REAL_CSV_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "time", "pitch_angle", "gyro_rate", "pwm_command", "enc_left", "enc_right", "fsm_state"
        ])
        writer.writeheader()
        writer.writerows(records)

    logger.success(f"Đã lưu {len(records)} mẫu dữ liệu thực tế vào: {REAL_CSV_FILE}")

    # Run auto_tune_pid.py
    print("\n" + logger.BOLD_CYAN + ">>> Đang kích hoạt nhận dạng ARX & tối ưu Nelder-Mead trên dữ liệu..." + logger.RESET)
    import subprocess
    cmd = [sys.executable, os.path.join(SCRIPT_DIR, "auto_tune_pid.py")]
    proc = subprocess.run(cmd)

    if proc.returncode == 0:
        logger.success("Tự động tinh chỉnh PID THÀNH CÔNG!")


def main_menu():
    logger.step_header("BỘ CÔNG CỤ TƯƠNG TÁC PHẦN CỨNG & AUTO-TUNE XE CÂN BẰNG")

    ser, port = connect_or_wait_serial()
    if ser is None:
        return

    try:
        while True:
            print("\n" + logger.BOLD_WHITE + "┌────────────────────────────────────────────────────────────┐" + logger.RESET)
            print(f"│ {logger.BOLD_CYAN}CỔNG ĐANG KẾT NỐI: {port:40s}{logger.RESET}{logger.BOLD_WHITE}│")
            print(logger.BOLD_WHITE + "├────────────────────────────────────────────────────────────┤" + logger.RESET)
            print(f"│  [1] Theo dõi Góc Nghiêng & MPU6050 (Live Pitch Visualizer) │")
            print(f"│  [2] Kiểm tra Chiều Đếm Encoder 2 Bánh (Live Wheel Counters)│")
            print(f"│  [3] Thử Nghiệm Xung Chiều Quay Động Cơ (Motor Dir Test)   │")
            print(f"│  [4] Căn Chỉnh Góc Cân Bằng Trọng Tâm (Zero Balance Trim)   │")
            print(f"│  [5] Thu Thập Dữ Liệu Thực Tế & Chạy Auto-Tune PID         │")
            print(f"│  [0] Thoát                                                 │")
            print(logger.BOLD_WHITE + "└────────────────────────────────────────────────────────────┘" + logger.RESET)

            choice = input(f"  Lựa chọn của bạn [0-5]: ").strip()
            if choice == "1":
                mode_live_pitch(ser)
            elif choice == "2":
                mode_live_encoders(ser)
            elif choice == "3":
                mode_test_motors(ser)
            elif choice == "4":
                mode_calibrate_zero(ser)
            elif choice == "5":
                mode_collect_telemetry_and_autotune(ser)
            elif choice == "0":
                break
            else:
                logger.warn("Lựa chọn không hợp lệ. Vui lòng chọn từ 0 đến 5.")
    finally:
        ser.close()
        logger.info("Đã đóng kết nối an toàn.")


if __name__ == "__main__":
    main_menu()

#!/usr/bin/env python3
"""
==============================================================================
Interactive & Automated Motor Deadzone Finder Tool for STM32 Self-Balancing Robot
Target: STM32F401CCU6 via ESP32-S3 BLE Bridge (/dev/ttyACM* or /dev/ttyUSB*)

Description:
This script allows user to manually or automatically ramp up PWM step-by-step
to visually observe when the GA25 motor begins turning, overcoming static friction.
Supports:
  1. Manual Step-by-Step Tuning (Arrow keys / W/S to increase slowly while watching wheels)
  2. Slow Auto-Ramp Mode (Increases by +5 PWM every 0.4s; press any key to freeze when wheel spins)
  3. Full Automated 4-Way Encoder Calibration
==============================================================================
"""

import sys
import os
import time
import glob
import re
import select
import serial
import argparse

# ANSI Colors & Formatting
CYAN = "\033[96m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"
CLEAR_LINE = "\033[2K\r"

BAUD = 115200

# Try importing termios & tty for non-blocking single-key detection
try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False


class KeyReader:
    """Non-blocking single character keyboard input handler for Linux."""
    def __init__(self):
        self.is_tty = sys.stdin.isatty() and HAS_TERMIOS
        self.old_settings = None
        if self.is_tty:
            try:
                self.fd = sys.stdin.fileno()
                self.old_settings = termios.tcgetattr(self.fd)
            except Exception:
                self.is_tty = False

    def enable_raw(self):
        if self.is_tty:
            try:
                tty.setcbreak(self.fd)
            except Exception:
                pass

    def restore(self):
        if self.is_tty and self.old_settings:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass

    def get_key(self, timeout=0.04):
        if not self.is_tty:
            return None
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if not rlist:
                return None
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                # Escape sequence for arrow keys
                r2, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r2:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        r3, _, _ = select.select([sys.stdin], [], [], 0.02)
                        if r3:
                            ch3 = sys.stdin.read(1)
                            if ch3 == 'A': return 'UP'
                            elif ch3 == 'B': return 'DOWN'
                            elif ch3 == 'C': return 'RIGHT'
                            elif ch3 == 'D': return 'LEFT'
                return 'ESC'
            elif ch in ['\r', '\n']:
                return 'ENTER'
            elif ch == ' ':
                return 'SPACE'
            elif ch == '\t':
                return 'TAB'
            return ch.lower()
        except Exception:
            return None


def find_serial_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if ports:
        return ports[0]
    return None


def connect_serial(port_name=None):
    if not port_name:
        port_name = find_serial_port()
    if not port_name:
        print(f"\n{RED}[LỖI KẾT NỐI]{RESET} Không tìm thấy cổng thiết bị (/dev/ttyACM* hoặc /dev/ttyUSB*).")
        print("  -> Vui lòng cắm cáp USB nối ESP32/STM32 vào máy tính và chạy lại lệnh!\n")
        return None, None
    try:
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = BAUD
        ser.timeout = 0.2
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(0.1)

        # Stop telemetry stream & bypass FSM to avoid overwriting PWM
        ser.write(b"STOP_STREAM\n")
        time.sleep(0.05)
        ser.write(b"MODE:MANUAL\n")
        time.sleep(0.05)
        ser.write(b"RAW_PWM:0,0\n")
        time.sleep(0.05)
        ser.reset_input_buffer()
        return ser, port_name
    except Exception as e:
        print(f"{RED}[LỖI] Không thể mở cổng {port_name}: {e}{RESET}")
        return None, None


def send_cmd(ser, cmd_str):
    if not ser:
        return ""
    try:
        ser.write((cmd_str.strip() + "\n").encode())
        time.sleep(0.03)
        return ser.readline().decode("utf-8", errors="ignore").strip()
    except Exception:
        return ""


def get_encoders(ser):
    if not ser:
        return None, None
    try:
        ser.reset_input_buffer()
        ser.write(b"GET_ENC\n")
        time.sleep(0.02)
        resp = ser.readline().decode("utf-8", errors="ignore").strip()
        if resp.startswith("ENC:"):
            parts = resp[4:].split(",")
            if len(parts) >= 2:
                return int(parts[0]), int(parts[1])
        # Fallback to STATUS command
        ser.write(b"STATUS\n")
        time.sleep(0.02)
        resp2 = ser.readline().decode("utf-8", errors="ignore").strip()
        m_l = re.search(r"ENC_L=([-\d]+)", resp2)
        m_r = re.search(r"ENC_R=([-\d]+)", resp2)
        if m_l and m_r:
            return int(m_l.group(1)), int(m_r.group(1))
    except Exception:
        pass
    return None, None


def stop_motors(ser):
    if not ser:
        return
    try:
        send_cmd(ser, "RAW_PWM:0,0")
        send_cmd(ser, "MOTOR:0,0")
        send_cmd(ser, "CMD:STOP")
        time.sleep(0.05)
    except Exception:
        pass


def set_raw_pwm(ser, pl, pr):
    if not ser:
        return
    send_cmd(ser, f"RAW_PWM:{pl},{pr}")


def update_header_deadzone(dz_val):
    header_path = "Modules/Inc/pid_AutoConfig.h"
    if not os.path.exists(header_path):
        return False
    try:
        with open(header_path, "r") as f:
            content = f.read()
        pattern = r"(#define\s+PWM_DEADZONE_DEFAULT\s+)\d+"
        new_content = re.sub(pattern, f"\\g<1>{dz_val}", content)
        with open(header_path, "w") as f:
            f.write(new_content)
        return True
    except Exception as e:
        print(f"[ERR] Không thể cập nhật {header_path}: {e}")
        return False


def handle_save_flow(ser, deadzone_val, description_str=""):
    print("\n\n" + "╔" + "═" * 66 + "╗")
    print(f"║ {BOLD}{YELLOW}           ĐÃ KHOÁ / CHỐT GIÁ TRỊ DEADZONE QUAN SÁT THỰC TẾ       {RESET} ║")
    print("╠" + "═" * 66 + "╣")
    if description_str:
        print(f"║  • Chế độ đo         : {CYAN}{description_str:<41}{RESET} ║")
    pct = (deadzone_val / 4199.0) * 100
    print(f"║  ★ {BOLD}DEADZONE CHỐT ĐƯỢC{RESET} : {BOLD}{GREEN}{deadzone_val:4d} PWM{RESET} ({pct:.1f}% công suất tối đa)           ║")
    print("╚" + "═" * 66 + "╝\n")

    print("  Chọn thao tác lưu trữ:")
    print(f"    [{BOLD}1{RESET} hoặc {BOLD}Y{RESET}]  -> {GREEN}NẠP NGAY{RESET} Deadzone = {deadzone_val} vào RAM STM32 qua ESP32")
    print(f"    [{BOLD}2{RESET}]       -> Nạp vào RAM {BOLD}VÀ{RESET} cập nhật file mã nguồn pid_AutoConfig.h")
    print(f"    [{BOLD}3{RESET} / {BOLD}ENTER{RESET}] -> Tiếp tục thử nghiệm (không lưu)")
    print(f"    [{BOLD}Q{RESET}]       -> Thoát chương trình\n")

    choice = input("  Nhập lựa chọn của bạn: ").strip().lower()
    if choice in ["1", "y", "yes"]:
        resp = send_cmd(ser, f"SET_DEADZONE:{deadzone_val}")
        print(f"\n  {GREEN}[THÀNH CÔNG]{RESET} Đã nạp vào RAM STM32: {resp}")
        return True
    elif choice == "2":
        resp = send_cmd(ser, f"SET_DEADZONE:{deadzone_val}")
        print(f"\n  {GREEN}[RAM THÀNH CÔNG]{RESET} Đã nạp vào RAM STM32: {resp}")
        if update_header_deadzone(deadzone_val):
            print(f"  {GREEN}[FILE C THÀNH CÔNG]{RESET} Đã ghi #define PWM_DEADZONE_DEFAULT {deadzone_val} vào Modules/Inc/pid_AutoConfig.h")
        return True
    return False


# ==============================================================================
# CHẾ ĐỘ 1: TĂNG THỦ CÔNG TỪNG BƯỚC BẰNG PHÍM (MANUAL STEP-BY-STEP)
# ==============================================================================
def run_manual_step_mode(ser):
    kr = KeyReader()
    kr.enable_raw()

    # Targets: 1=Left, 2=Right, 3=Both
    target_mode = 3
    dir_sign = +1
    current_pwm = 0
    max_pwm = 2000

    enc_l_init, enc_r_init = get_encoders(ser)
    if enc_l_init is None:
        enc_l_init, enc_r_init = 0, 0
    base_l = enc_l_init
    base_r = enc_r_init

    print("\n" + "╔" + "═" * 68 + "╗")
    print(f"║ {BOLD}{YELLOW}     CHẾ ĐỘ TĂNG TỪ TỪ THỦ CÔNG ĐỂ QUAN SÁT TẬN MẮT BÁNH XE QUAY    {RESET} ║")
    print("╠" + "═" * 68 + "╣")
    print("║ • Bấm phím để tăng/giảm PWM từ từ. Nhìn bánh xe xem khi nào bắt đầu quay ║")
    print("║ • [↑] / [W]   : Tăng +10 PWM          [↓] / [S]   : Giảm -10 PWM           ║")
    print("║ • [→] / [D]   : Tăng nhanh (+50)      [←] / [A]   : Giảm nhanh (-50)       ║")
    print("║ • [1] / [2]   : Bánh Trái / Phải      [3]         : Cả 2 Bánh              ║")
    print("║ • [T] / [Tab] : Đổi chiều (Tiến/Lùi)  [Space]     : Dừng khẩn cấp (=0 PWM) ║")
    print("║ • [V]         : Nhập trực tiếp số PWM từ bàn phím                          ║")
    print("║ • [ENTER]     : ★ CHỐT DEADZONE tại mức này & lưu vào cấu hình             ║")
    print("║ • [Q]         : Thoát về menu chính                                        ║")
    print("╚" + "═" * 68 + "╝\n")

    try:
        last_disp_time = 0
        while True:
            key = kr.get_key(timeout=0.05)

            if key:
                if key in ['up', 'w', '+']:
                    current_pwm = min(max_pwm, current_pwm + 10)
                elif key in ['down', 's', '-']:
                    current_pwm = max(0, current_pwm - 10)
                elif key in ['right', 'd']:
                    current_pwm = min(max_pwm, current_pwm + 50)
                elif key in ['left', 'a']:
                    current_pwm = max(0, current_pwm - 50)
                elif key == '1':
                    target_mode = 1
                    base_l, base_r = get_encoders(ser) or (0, 0)
                elif key == '2':
                    target_mode = 2
                    base_l, base_r = get_encoders(ser) or (0, 0)
                elif key == '3':
                    target_mode = 3
                    base_l, base_r = get_encoders(ser) or (0, 0)
                elif key in ['t', 'tab']:
                    dir_sign = -dir_sign
                elif key == 'space':
                    current_pwm = 0
                elif key == 'v':
                    # Direct value input
                    kr.restore()
                    stop_motors(ser)
                    try:
                        val_str = input(f"\n  Nhập mức PWM muốn thử (0..{max_pwm}): ").strip()
                        if val_str.isdigit():
                            current_pwm = int(val_str)
                    except Exception:
                        pass
                    kr.enable_raw()
                    base_l, base_r = get_encoders(ser) or (0, 0)
                elif key == 'enter':
                    # Freeze & Save
                    kr.restore()
                    stop_motors(ser)
                    mode_name = ("Bánh TRÁI" if target_mode == 1 else ("Bánh PHẢI" if target_mode == 2 else "CẢ 2 BÁNH"))
                    dir_name = "TIẾN" if dir_sign > 0 else "LÙI"
                    handle_save_flow(ser, current_pwm, f"{mode_name} - {dir_name}")
                    return
                elif key in ['q', 'esc']:
                    break

                # Apply PWM to hardware
                pwm_l = (current_pwm * dir_sign) if target_mode in [1, 3] else 0
                pwm_r = (current_pwm * dir_sign) if target_mode in [2, 3] else 0
                set_raw_pwm(ser, pwm_l, pwm_r)

            # Update live telemetry display at ~10Hz
            now = time.time()
            if now - last_disp_time > 0.08:
                last_disp_time = now
                el, er = get_encoders(ser)
                if el is not None and er is not None:
                    delta_l = abs(el - base_l)
                    delta_r = abs(er - base_r)
                else:
                    delta_l, delta_r = 0, 0

                active_delta = max(delta_l, delta_r) if target_mode == 3 else (delta_l if target_mode == 1 else delta_r)
                
                # Visual Bar
                bar_len = 16
                filled = int((current_pwm / max_pwm) * bar_len)
                bar = "█" * filled + "░" * (bar_len - filled)

                # Motor string
                motor_str = "BÁNH TRÁI" if target_mode == 1 else ("BÁNH PHẢI" if target_mode == 2 else "CẢ 2 BÁNH")
                dir_str = "TIẾN" if dir_sign > 0 else "LÙI"

                # Status
                if current_pwm == 0:
                    status_str = f"{DIM}[ĐANG TẮT]{RESET}"
                elif active_delta >= 4:
                    status_str = f"{BOLD}{GREEN}[ĐANG QUAY! ✔]{RESET}"
                elif active_delta >= 1:
                    status_str = f"{YELLOW}[NHÚC NHÍCH...]{RESET}"
                else:
                    status_str = f"{YELLOW}[ĐỨNG YÊN]{RESET}"

                sys.stdout.write(
                    f"{CLEAR_LINE}  PWM: {BOLD}{CYAN}{current_pwm:4d}{RESET} [{bar}] | "
                    f"{BOLD}{motor_str} ({dir_str}){RESET} | "
                    f"ΔXung: {BOLD}{active_delta:2d}{RESET} | {status_str} "
                )
                sys.stdout.flush()

    finally:
        kr.restore()
        stop_motors(ser)
        print(f"\n{GREEN}[AN TOÀN]{RESET} Đã ngắt PWM động cơ về 0.\n")


# ==============================================================================
# CHẾ ĐỘ 2: TỰ ĐỘNG TĂNG CHẬM - BẤM PHÍM BẤT KỲ ĐỂ DỪNG KHI THẤY BÁNH QUAY
# ==============================================================================
def run_slow_auto_ramp_mode(ser):
    kr = KeyReader()

    print("\n" + "╔" + "═" * 68 + "╗")
    print(f"║ {BOLD}{YELLOW}        CHẾ ĐỘ TỰ ĐỘNG TĂNG CHẬM (+5 PWM MỖI 0.35 GIÂY)            {RESET} ║")
    print("╠" + "═" * 68 + "╣")
    print("║ • Chương trình sẽ tăng PWM từ từ một cách cực kỳ êm ái.                    ║")
    print("║ • {BOLD}Bạn chỉ việc nhìn vào bánh xe.{RESET}                                        ║")
    print(f"║ • Ngay khi thấy bánh xe {BOLD}{GREEN}BẮT ĐẦU QUAY{RESET}, bấm {BOLD}{GREEN}PHÍM CÁCH (SPACE) HOẶC ENTER{RESET}  ║")
    print("║   để lập tức ngắt động cơ và chốt ngay mức Deadzone đó!                    ║")
    print("╚" + "═" * 68 + "╝\n")

    print("  Chọn động cơ muốn thử:")
    print("    [1] Bánh Trái (Tiến)")
    print("    [2] Bánh Phải (Tiến)")
    print("    [3] Cả 2 Bánh (Tiến) [Khuyên dùng]")
    print("    [4] Cả 2 Bánh (Lùi)")
    choice = input("  Lựa chọn (1..4, mặc định=3): ").strip()

    target_mode = 3
    dir_sign = +1
    if choice == "1":
        target_mode = 1
    elif choice == "2":
        target_mode = 2
    elif choice == "4":
        dir_sign = -1

    input(f"\n  Nhấn {BOLD}{GREEN}ENTER{RESET} để bắt đầu tăng tốc từ từ...")

    kr.enable_raw()
    stop_motors(ser)
    time.sleep(0.1)

    enc_l_init, enc_r_init = get_encoders(ser) or (0, 0)
    base_l = enc_l_init
    base_r = enc_r_init

    max_pwm = 1500
    step = 5
    step_delay = 0.35  # Tăng mỗi 0.35s để mắt quan sát rõ ràng
    detected_pwm = None

    try:
        for pwm_val in range(20, max_pwm + 1, step):
            # Check for user keypress (any key to freeze)
            key = kr.get_key(timeout=0.01)
            if key:
                detected_pwm = pwm_val
                stop_motors(ser)
                break

            # Apply PWM
            pwm_l = (pwm_val * dir_sign) if target_mode in [1, 3] else 0
            pwm_r = (pwm_val * dir_sign) if target_mode in [2, 3] else 0
            set_raw_pwm(ser, pwm_l, pwm_r)

            # Read encoder
            el, er = get_encoders(ser)
            dl = abs(el - base_l) if el is not None else 0
            dr = abs(er - base_r) if er is not None else 0
            active_delta = max(dl, dr) if target_mode == 3 else (dl if target_mode == 1 else dr)

            bar_len = 16
            filled = int((pwm_val / max_pwm) * bar_len)
            bar = "█" * filled + "░" * (bar_len - filled)

            status_str = f"{BOLD}{GREEN}[BẮT ĐẦU QUAY!]{RESET}" if active_delta >= 3 else f"{YELLOW}[Đang tăng...]{RESET}"

            sys.stdout.write(
                f"{CLEAR_LINE}  PWM: {BOLD}{CYAN}{pwm_val:4d}{RESET} [{bar}] | "
                f"ΔXung: {BOLD}{active_delta:2d}{RESET} | {status_str} "
                f"(Bấm {BOLD}SPACE{RESET} để dừng) "
            )
            sys.stdout.flush()

            # Automatic stop if encoder rotates heavily (> 6 pulses)
            if active_delta >= 6:
                detected_pwm = pwm_val
                stop_motors(ser)
                print(f"\n\n  {GREEN}✔ ĐÃ TỰ ĐỘNG PHÁT HIỆN BÁNH XE QUAY TẠI: {BOLD}{pwm_val} PWM{RESET} (ΔXung = {active_delta})")
                break

            time.sleep(step_delay)

    finally:
        kr.restore()
        stop_motors(ser)

    if detected_pwm is not None:
        mode_name = ("Bánh TRÁI" if target_mode == 1 else ("Bánh PHẢI" if target_mode == 2 else "CẢ 2 BÁNH"))
        dir_name = "TIẾN" if dir_sign > 0 else "LÙI"
        handle_save_flow(ser, detected_pwm, f"{mode_name} - {dir_name}")
    else:
        print("\n  Đã dừng mà không ghi nhận giá trị.")


# ==============================================================================
# CHẾ ĐỘ 3: QUÉT TỰ ĐỘNG 4 CHIỀU TOÀN DIỆN (AUTOMATED 4-WAY)
# ==============================================================================
def probe_single_motor_auto(ser, motor_name, is_left, direction_sign, max_pwm=700, step=10, step_delay=0.10):
    dir_str = "TIẾN (FWD)" if direction_sign > 0 else "LÙI (REV)"
    print(f"\n  ► {BOLD}Đang đo {motor_name} - Chiều {dir_str}...{RESET}")

    stop_motors(ser)
    time.sleep(0.2)

    enc_l_init, enc_r_init = get_encoders(ser) or (0, 0)
    base_enc = enc_l_init if is_left else enc_r_init
    detected_pwm = None

    for pwm_mag in range(20, max_pwm + 1, step):
        target_pwm = pwm_mag * direction_sign
        if is_left:
            set_raw_pwm(ser, target_pwm, 0)
        else:
            set_raw_pwm(ser, 0, target_pwm)

        time.sleep(step_delay)

        el, er = get_encoders(ser)
        if el is None:
            continue
        curr_enc = el if is_left else er
        delta_pulses = abs(curr_enc - base_enc)

        pct = int((pwm_mag / max_pwm) * 20)
        bar = "█" * pct + "░" * (20 - pct)
        sys.stdout.write(f"{CLEAR_LINE}    [PWM: {pwm_mag:3d} / {max_pwm}] [{bar}] ΔXung: {delta_pulses:2d} ")
        sys.stdout.flush()

        if delta_pulses >= 4:
            detected_pwm = pwm_mag
            stop_motors(ser)
            print(f"\n    {GREEN}✔ PHÁT HIỆN CHUYỂN ĐỘNG!{RESET} Ngưỡng Deadzone = {BOLD}{GREEN}{detected_pwm} PWM{RESET} (ΔXung = {delta_pulses})")
            break

    stop_motors(ser)
    if detected_pwm is None:
        print(f"\n    {YELLOW}⚠ Không phát hiện chuyển động trong dải 0..{max_pwm} PWM.{RESET}")
        detected_pwm = max_pwm

    time.sleep(0.3)
    return detected_pwm


def run_full_auto_mode(ser):
    print("\n  Bắt đầu quá trình đo tự động 4 chiều (Trái/Phải - Tiến/Lùi)...")
    dz_l_fwd = probe_single_motor_auto(ser, "BÁNH TRÁI", is_left=True, direction_sign=+1)
    dz_l_rev = probe_single_motor_auto(ser, "BÁNH TRÁI", is_left=True, direction_sign=-1)
    dz_r_fwd = probe_single_motor_auto(ser, "BÁNH PHẢI", is_left=False, direction_sign=+1)
    dz_r_rev = probe_single_motor_auto(ser, "BÁNH PHẢI", is_left=False, direction_sign=-1)

    dz_left_avg = int((dz_l_fwd + dz_l_rev) / 2)
    dz_right_avg = int((dz_r_fwd + dz_r_rev) / 2)
    dz_optimal = int((dz_left_avg + dz_right_avg) / 2)

    print("\n\n" + "╔" + "═" * 66 + "╗")
    print(f"║ {BOLD}{YELLOW}               TỔNG KẾT KẾT QUẢ ĐO DEADZONE ĐỘNG CƠ              {RESET} ║")
    print("╠" + "═" * 66 + "╣")
    print(f"║  • Bánh TRÁI  : Tiến = {dz_l_fwd:3d} PWM  |  Lùi = {dz_l_rev:3d} PWM  |  TB = {BOLD}{dz_left_avg:3d} PWM{RESET}  ║")
    print(f"║  • Bánh PHẢI  : Tiến = {dz_r_fwd:3d} PWM  |  Lùi = {dz_r_rev:3d} PWM  |  TB = {BOLD}{dz_right_avg:3d} PWM{RESET}  ║")
    print("╠" + "═" * 66 + "╣")
    print(f"║  ★ {BOLD}DEADZONE TỐI ƯU TOÀN XE{RESET} : {BOLD}{GREEN}{dz_optimal:3d} PWM{RESET} (Dải khuyên dùng: {dz_optimal-15}..{dz_optimal+15})   ║")
    print("╚" + "═" * 66 + "╝\n")

    handle_save_flow(ser, dz_optimal, "Quét tự động 4 chiều")


# ==============================================================================
# MAIN FUNCTION
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Motor Deadzone Finder Tool")
    parser.add_argument("--port", default=None, help="Serial port (/dev/ttyACM0, /dev/ttyUSB0, etc.)")
    args = parser.parse_args()

    print("\n" + "═" * 72)
    print(f" {BOLD}CÔNG CỤ TÌM VÙNG CHẾT ĐỘNG CƠ GA25 (MOTOR DEADZONE FINDER){RESET} ")
    print(" Target: STM32F401CCU6 qua ESP32-S3 BLE Bridge (/dev/ttyACM*)")
    print("═" * 72)
    print(" MÔ TẢ:")
    print(" • Do ma sát cơ học của hộp số GA25, ở mức PWM nhỏ bánh xe sẽ không quay.")
    print(" • Bạn có thể tự tăng/giảm PWM từng bước và nhìn tận mắt khi nào bánh quay.")
    print("═" * 72 + "\n")

    ser, port_name = connect_serial(args.port)
    if not ser:
        sys.exit(1)

    print(f"  [KẾT NỐI] Cổng: {port_name} @ {BAUD} bps")

    # Read current deadzone in STM32
    resp_dz = send_cmd(ser, "GET_DEADZONE")
    curr_dz = resp_dz.split(":")[1] if ":" in resp_dz else "220"
    print(f"  [HIỆN TẠI] Deadzone đang lưu trong STM32: {BOLD}{curr_dz} PWM{RESET}\n")

    try:
        while True:
            print("  CHỌN PHƯƠNG THỨC ĐO:")
            print(f"    [{BOLD}1{RESET} hoặc {BOLD}ENTER{RESET}] -> {GREEN}{BOLD}CHỈNH TAY BẰNG PHÍM NÓNG{RESET} (Tự tăng/giảm từ từ để nhìn bánh xe)")
            print(f"    [{BOLD}2{RESET}]             -> {CYAN}TỰ ĐỘNG TĂNG CHẬM{RESET} (+5 PWM mỗi 0.35s, bấm phím dừng khi bánh quay)")
            print(f"    [{BOLD}3{RESET}]             -> {YELLOW}QUÉT TỰ ĐỘNG 4 CHIỀU{RESET} (Đo bằng cảm biến Encoder)")
            print(f"    [{BOLD}Q{RESET}]             -> Thoát chương trình\n")

            choice = input("  Nhập lựa chọn của bạn: ").strip().lower()

            if choice in ["1", ""]:
                run_manual_step_mode(ser)
            elif choice == "2":
                run_slow_auto_ramp_mode(ser)
            elif choice == "3":
                run_full_auto_mode(ser)
            elif choice in ["q", "quit", "exit"]:
                break
            else:
                print(f"  Lựa chọn không hợp lệ: {choice}\n")

    except KeyboardInterrupt:
        print(f"\n\n{YELLOW}[HỦY BỎ]{RESET} Nhận tín hiệu ngắt từ người dùng.")
    finally:
        stop_motors(ser)
        send_cmd(ser, "MODE:AUTO")  # Re-enable balancing mode upon exit
        ser.close()
        print(f"\n{GREEN}[HOÀN TẤT]{RESET} Đã ngắt động cơ an toàn và hoàn trả chế độ AUTO.\n")


if __name__ == "__main__":
    main()

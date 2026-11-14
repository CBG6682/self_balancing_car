#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
 Công Cụ Chẩn Đoán Phần Cứng Động Cơ & Mạch Cầu H TB6612 (Motor Hardware Probe)
==============================================================================
"""

import os
import sys

# Tự động nạp môi trường ảo (venv) nếu chạy bằng python3 mặc định của hệ thống
def _bootstrap_venv():
    try:
        import serial
        return
    except ImportError:
        pass

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    venv_py = os.path.join(repo_root, "venv", "bin", "python3")
    if os.path.isfile(venv_py) and os.path.abspath(sys.executable) != os.path.abspath(venv_py):
        os.execv(venv_py, [venv_py] + sys.argv)

    import glob
    for sp in glob.glob(os.path.join(repo_root, "venv", "lib", "python*", "site-packages")):
        if os.path.isdir(sp) and sp not in sys.path:
            sys.path.insert(0, sp)

_bootstrap_venv()

import time
import serial
import serial.tools.list_ports

try:
    from logger_utils import ColoredLogger
    logger = ColoredLogger("MOTOR_PROBE")
except ImportError:
    class DummyLogger:
        def info(self, m): print(f"[INFO] {m}")
        def success(self, m): print(f"\033[92m[PASS] {m}\033[0m")
        def warn(self, m): print(f"\033[93m[WARN] {m}\033[0m")
        def error(self, m): print(f"\033[91m[FAIL] {m}\033[0m")
        def sub_header(self, m): print(f"\n--- {m} ---")
    logger = DummyLogger()

BAUD = 115200


def find_serial_port():
    """Tìm cổng Serial vật lý của ESP32 hoặc bộ chuyển đổi USB-UART."""
    ports = list(serial.tools.list_ports.comports())
    
    # 1. Tìm ESP32-S3 (VID=0x303A)
    for p in ports:
        if p.vid == 0x303A:
            return p.device

    # 2. Tìm thiết bị /dev/ttyACM* hoặc /dev/ttyUSB*
    for p in ports:
        dev = p.device
        if any(dev.startswith(pfx) for pfx in ["/dev/ttyACM", "/dev/ttyUSB", "/dev/cu.usb"]):
            return dev

    # 3. Quét trực tiếp file hệ thống Linux
    for i in range(4):
        dev = f"/dev/ttyACM{i}"
        if os.path.exists(dev):
            return dev
    for i in range(4):
        dev = f"/dev/ttyUSB{i}"
        if os.path.exists(dev):
            return dev

    return None


def wait_for_serial_port():
    """Chờ cho đến khi người dùng cắm cáp USB vào máy tính."""
    port = find_serial_port()
    if port:
        return port

    print("\n" + "!" * 65)
    print(" [CẢNH BÁO] CHƯA PHÁT HIỆN CỔNG USB CỦA ESP32 / STM32!")
    print(" Vui lòng kiểm tra và cắm cáp USB của ESP32-S3 vào máy tính...")
    print(" Đang tự động quét tìm cổng... (Nhấn Ctrl+C để hủy)")
    print("!" * 65 + "\n")

    try:
        dot_cnt = 0
        while True:
            port = find_serial_port()
            if port:
                print(f"\n\033[92m  ✔ Đã phát hiện cổng: {port}\033[0m")
                return port
            dot_cnt += 1
            print(f"\r  Đang tìm thiết bị USB{'.' * (dot_cnt % 6 + 1)}      ", end="", flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[INFO] Đã hủy bởi người dùng.")
        sys.exit(0)


def print_hardware_pin_table():
    print("\n" + "=" * 70)
    print(" SƠ ĐỒ ĐỐI CHIẾU DÂY NỐI: BLACKPILL (STM32F401) <---> TB6612FNG")
    print("=" * 70)
    print("  TB6612 Pin  | Nối vào Chân STM32 / Nguồn  | Điện áp chuẩn khi TEST")
    print(" ─────────────┼─────────────────────────────┼─────────────────────────")
    print("  VM          | Cực Dương (+) Khối PIN      | 7.4V - 8.4V (Pin 2S)")
    print("  VCC         | Chân 3.3V (hoặc 5V) STM32   | 3.3V (hoặc 5.0V)")
    print("  GND         | GND STM32 & Cực Âm (-) PIN  | 0.0V (Nối đất chung)")
    print("  STBY        | PB0 (hoặc nối thẳng 3.3V)   | 3.3V (Mức CAO kích hoạt)")
    print(" ─────────────┼─────────────────────────────┼─────────────────────────")
    print("  PWMA        | PB8 (TIM4_CH3)              | ~1.2V - 2.5V (Xung PWM)")
    print("  AIN1        | PA4                         | 3.3V (Khi quay TIẾN)")
    print("  AIN2        | PA5                         | 0.0V (Khi quay TIẾN)")
    print("  AO1, AO2    | 2 Dây Motor Trái (GA25)     | Đo giữa 2 cọc: ~4V - 7V")
    print(" ─────────────┼─────────────────────────────┼─────────────────────────")
    print("  PWMB        | PB9 (TIM4_CH4)              | ~1.2V - 2.5V (Xung PWM)")
    print("  BIN1        | PA6                         | 3.3V (Khi quay TIẾN)")
    print("  BIN2        | PA7                         | 0.0V (Khi quay TIẾN)")
    print("  BO1, BO2    | 2 Dây Motor Phải (GA25)     | Đo giữa 2 cọc: ~4V - 7V")
    print("=" * 70)


def send_continuous_motor_pulse(ser, cmd_str, duration_s=10.0):
    """Giữ phát xung liên tục trong duration_s bằng cách gửi lặp lại xung an toàn."""
    print(f"\n[*] Bắt đầu phát xung: {cmd_str}")
    print(f"[*] Đang DUY TRÌ tín hiệu trong {duration_s:.0f}s để bạn dùng đồng hồ VOM đo điện áp...")
    print("    (Nhấn Ctrl+C bất kỳ lúc nào để ngắt khẩn cấp)\n")

    t_start = time.time()
    last_send = 0

    try:
        while time.time() - t_start < duration_s:
            now = time.time()
            rem = duration_s - (now - t_start)

            # Gửi lặp lại mỗi 500ms để giữ mạch cầu H luôn mở
            if now - last_send >= 0.5:
                ser.write((cmd_str + "\n").encode("utf-8"))
                ser.flush()
                last_send = now

            while ser.in_waiting:
                line = ser.readline().decode("latin1", errors="ignore").strip()
                if line and not line.startswith("DATA:"):
                    print(f"    [STM32]: {line}")

            print(f"    -> Đang phát tín hiệu... còn lại {rem:4.1f}s (Đang kích điện áp)", end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[!] Ngắt sớm bởi người dùng.")

    print("\n[*] Dừng phát xung: TEST:STOP")
    ser.write(b"TEST:STOP\n")
    ser.flush()
    time.sleep(0.2)
    while ser.in_waiting:
        line = ser.readline().decode("latin1", errors="ignore").strip()
        if line:
            print(f"    [STM32]: {line}")


def test_hand_spin_encoders(ser):
    print("\n" + "=" * 70)
    print(" KIỂM TRA ĐỘNG CƠ CƠ KHÍ & ENCODER (XOAY BÁNH XE BẰNG TAY)")
    print("=" * 70)
    print(" Hướng dẫn:")
    print("  1. Dùng tay nhẹ nhàng xoay bánh xe TRÁI hoặc bánh xe PHẢI.")
    print("  2. Quan sát giá trị xung ENC_L và ENC_R hiển thị bên dưới.")
    print("  3. Nếu lấy tay xoay mà giá trị ENC nhảy số liên tục -> Cơ khí không kẹt,")
    print("     Encoder tốt, dây tín hiệu về STM32 tốt.")
    print("  4. Nếu lấy tay xoay thấy rất nặng/kẹt cứng -> Động cơ hoặc hộp số bị kẹt.")
    print(" Nhấn Ctrl+C để kết thúc bài kiểm tra này.\n")

    try:
        ser.reset_input_buffer()
        ser.write(b"START_STREAM\n")
        ser.flush()
        last_enc_l = None
        last_enc_r = None

        while True:
            line = ser.readline().decode("latin1", errors="ignore").strip()
            if line.startswith("DATA:"):
                parts = line.split(":")[1].split(",")
                if len(parts) >= 5:
                    try:
                        enc_l = int(parts[3])
                        enc_r = int(parts[4])
                        pitch = float(parts[0])

                        delta_l = f"({enc_l - last_enc_l:+d})" if last_enc_l is not None else ""
                        delta_r = f"({enc_r - last_enc_r:+d})" if last_enc_r is not None else ""
                        last_enc_l = enc_l
                        last_enc_r = enc_r

                        print(f"  [ENCODER LIVE] Bánh Trái (TIM2): {enc_l:6d} {delta_l:7s} | Bánh Phải (TIM3): {enc_r:6d} {delta_r:7s} | Góc Nghiêng: {pitch:5.1f}°", end="\r", flush=True)
                    except ValueError:
                        pass
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\n[INFO] Đã dừng đọc Encoder.")
    finally:
        ser.write(b"STOP_STREAM\n")
        ser.flush()


def open_serial_safe(port_name):
    """Mở cổng Serial an toàn với xử lý ngoại lệ rõ ràng."""
    try:
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = BAUD
        ser.timeout = 0.2
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(1.0)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        logger.error(f"Không thể mở cổng {port_name}: {e}")
        return None


def main():
    print_hardware_pin_table()

    # Tự động tìm hoặc chờ cắm cáp USB
    port = wait_for_serial_port()

    print(f"\n[KẾT NỐI] Đang mở cổng {port} @ {BAUD} bps...")
    ser = open_serial_safe(port)
    if not ser:
        print("\n[!] Không thể kết nối. Vui lòng rút cáp USB cắm lại và chạy lại script.")
        sys.exit(1)

    # Thử bắt tay PING -> PONG
    ser.write(b"PING\n")
    time.sleep(0.15)
    resp = ser.read_all().decode("latin1", errors="ignore")
    if "PONG" in resp:
        logger.success("Kết nối thành công tới STM32 BlackPill (PING -> PONG OK)!")
    else:
        logger.warn(f"Cổng mở được, phản hồi khởi đầu: {resp.strip() if resp.strip() else '(Trống)'}")

    try:
        while True:
            print("\n" + "═" * 58)
            print(" MENU CHẨN ĐOÁN PHẦN CỨNG ĐỘNG CƠ")
            print("═" * 58)
            print("  [1] Giữ Motor TRÁI chạy TIẾN (10 giây) để đo áp que đo")
            print("  [2] Giữ Motor PHẢI chạy TIẾN (10 giây) để đo áp que đo")
            print("  [3] Giữ CẢ 2 MOTOR chạy TIẾN (10 giây) để đo áp que đo")
            print("  [4] Xoay bánh xe bằng tay (Kiểm tra kẹt cơ khí & Encoder)")
            print("  [5] Xem lại Bảng đối chiếu sơ đồ chân (Pinout Table)")
            print("  [s] Dừng khẩn cấp (Emergency Stop)")
            print("  [q] Thoát chương trình")
            print("═" * 58)

            try:
                choice = input("Nhập lựa chọn của bạn [1-5, s, q]: ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == "1":
                print("\n[ĐO KIỂM BÁNH TRÁI]:")
                print(" -> Chân VM: Đo que đỏ vào VM, que đen vào GND -> Phải có 7.4V - 8.4V")
                print(" -> Chân STBY: Đo que đỏ vào STBY -> Phải có 3.3V")
                print(" -> Chân PWMA (PB8): Que đỏ vào PB8 -> Phải có ~1.5V - 2.5V")
                print(" -> Chân AIN1 (PA4): Phải có 3.3V | Chân AIN2 (PA5): Phải có 0V")
                print(" -> Đầu ra AO1 - AO2: Đo giữa 2 ốc domino -> Phải có ~4V - 7V")
                send_continuous_motor_pulse(ser, "TEST:MOTOR_L", duration_s=10.0)
            elif choice == "2":
                print("\n[ĐO KIỂM BÁNH PHẢI]:")
                print(" -> Chân VM: Đo que đỏ vào VM, que đen vào GND -> Phải có 7.4V - 8.4V")
                print(" -> Chân STBY: Đo que đỏ vào STBY -> Phải có 3.3V")
                print(" -> Chân PWMB (PB9): Que đỏ vào PB9 -> Phải có ~1.5V - 2.5V")
                print(" -> Chân BIN1 (PA6): Phải có 3.3V | Chân BIN2 (PA7): Phải có 0V")
                print(" -> Đầu ra BO1 - BO2: Đo giữa 2 ốc domino -> Phải có ~4V - 7V")
                send_continuous_motor_pulse(ser, "TEST:MOTOR_R", duration_s=10.0)
            elif choice == "3":
                send_continuous_motor_pulse(ser, "TEST:MOTORS", duration_s=10.0)
            elif choice == "4":
                test_hand_spin_encoders(ser)
            elif choice == "5":
                print_hardware_pin_table()
            elif choice == "s":
                ser.write(b"TEST:STOP\n")
                time.sleep(0.1)
                logger.warn("Đã gửi TEST:STOP.")
            elif choice == "q":
                break
            else:
                print("Lựa chọn không hợp lệ.")

    finally:
        try:
            ser.write(b"TEST:STOP\n")
            ser.write(b"STOP_STREAM\n")
        except:
            pass
        ser.close()
        logger.info("Đã đóng kết nối cổng Serial an toàn.")


if __name__ == "__main__":
    main()

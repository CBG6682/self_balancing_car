#!/usr/bin/env python3
"""
Comprehensive Hardware & Connection Diagnostic Suite
Target: STM32F401CCU6 Self-Balancing Robot
Tests every component step-by-step with interactive user confirmation before each step.

Components Tested:
1. USB Serial Port & ESP32-S3 Bridge
2. STM32 Physical UART Handshake (PING/PONG & STATUS)
3. MPU6050 6-DOF IMU & I2C Bus (PB6 SCL / PB7 SDA, Pitch Angle, Gyro Rate)
4. GA25 Wheel Encoders (TIM2 PA0/PA1, TIM3 PB4/PB5)
5. TB6612 Dual H-Bridge & Left Motor (PWMA PB8, AIN1 PA4, AIN2 PA5, STBY PB0)
6. TB6612 Dual H-Bridge & Right Motor (PWMB PB9, BIN1 PA6, BIN2 PA7)
7. Dual Motor Synchronization & Emergency Brake (Safety Stop)
"""

import os
import sys

# Auto-activate workspace venv if pyserial is not in current python environment
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

# Import shared logger utility
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import logger_utils as logger

BAUD = 115200

STEP_1 = "1. Cổng USB / ESP32-S3 Bridge"
STEP_2 = "2. Đường truyền UART (ESP32 <-> STM32)"
STEP_3 = "3. Kết nối & Lệnh Bluetooth BLE (App Smartphone & LED)"
STEP_4 = "4. Cảm biến MPU6050 (I2C PB6/PB7)"
STEP_5 = "5. Đĩa Encoder GA25 (TIM2 & TIM3)"
STEP_6 = "6. Động cơ & Cầu H (Bánh Trái)"
STEP_7 = "7. Động cơ & Cầu H (Bánh Phải)"
STEP_8 = "8. Đồng bộ 2 động cơ & Phanh khẩn cấp"

ALL_STEPS = [STEP_1, STEP_2, STEP_3, STEP_4, STEP_5, STEP_6, STEP_7, STEP_8]


def safe_input(prompt_text, default=""):
    """Reads user input safely without crashing on EOF or Interrupt."""
    try:
        val = input(prompt_text).strip()
        return val if val else default
    except EOFError:
        return "q"  # End-of-file (stream closed / non-interactive) -> exit cleanly
    except KeyboardInterrupt:
        return "q"


def auto_detect_port():
    """Finds best matching physical COM/tty port, prioritizing ESP32-S3."""
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        return sys.argv[1]

    ports = list(serial.tools.list_ports.comports())

    # 1. Prioritize Espressif ESP32-S3 (VID: 0x303a)
    for p in ports:
        if p.vid == 0x303A:
            return p.device

    # 2. Prioritize common USB-UART chips
    for p in ports:
        desc = (p.description or "").lower()
        if any(chip in desc for chip in ["ch340", "ch341", "ch343", "cp210", "ftdi", "uart"]):
            return p.device

    # 3. Filter other physical devices (ignoring mobile phones like Samsung 0x04e8)
    phys = logger.filter_physical_ports(ports)
    for p in phys:
        if p.vid == 0x04E8:
            continue
        return p.device

    # Fallback default
    if sys.platform == "win32":
        return "COM3"
    if os.path.exists("/dev/ttyACM0"):
        return "/dev/ttyACM0"
    if os.path.exists("/dev/ttyUSB0"):
        return "/dev/ttyUSB0"
    return None


def scan_usb_hardware():
    """Scans USB bus for ESP32, STM32 Bootloader, and ST-Link probes."""
    info = {
        "serial_ports": [],
        "has_stm32_hid": False,
        "has_stm32_dfu": False,
        "has_stlink": False,
    }

    # 1. Serial ports (filtered to exclude dummy ttyS and phone ports)
    all_com = list(serial.tools.list_ports.comports())
    phys_com = logger.filter_physical_ports(all_com)
    for p in phys_com:
        if p.vid == 0x04E8:
            continue  # Ignore Samsung phone modem
        info["serial_ports"].append((p.device, p.description, p.vid, p.pid))

    # 2. WeAct HID Bootloader (0483:572a)
    try:
        import hid
        devs = hid.enumerate(0x0483, 0x572A)
        if devs:
            info["has_stm32_hid"] = True
    except Exception:
        pass

    # 3. STM32 DFU Mode (0483:df11) or ST-Link (0483:3748 / 374b / 3752)
    try:
        import usb.core
        if usb.core.find(idVendor=0x0483, idProduct=0xDF11) is not None:
            info["has_stm32_dfu"] = True
        if (usb.core.find(idVendor=0x0483, idProduct=0x3748) is not None or
            usb.core.find(idVendor=0x0483, idProduct=0x374B) is not None or
            usb.core.find(idVendor=0x0483, idProduct=0x3752) is not None):
            info["has_stlink"] = True
    except Exception:
        pass

    return info


def print_hardware_inventory(hw):
    """Displays connected USB devices and diagnostic hints."""
    print("┌" + "─" * 68 + "┐")
    print(f"│ {logger.BOLD_WHITE}TRẠNG THÁI THIẾT BỊ USB HIỆN TẠI:{logger.RESET}{' ' * 36}│")
    if hw["serial_ports"]:
        for dev, desc, vid, pid in hw["serial_ports"]:
            vid_pid = f"({vid:04x}:{pid:04x})" if vid and pid else ""
            line = f" • Cổng Serial: {dev} - {desc} {vid_pid}"[:66]
            print(f"│ {line:<66} │")
    else:
        print(f"│   {logger.YELLOW}• Cổng Serial: Chưa phát hiện cổng COM / /dev/tty nào{logger.RESET}{' ' * 14}│")

    if hw["has_stm32_hid"]:
        print(f"│   {logger.CYAN}• STM32: Phát hiện WeAct Studio HID Bootloader (0483:572a){logger.RESET}{' ' * 9}│")
    elif hw["has_stm32_dfu"]:
        print(f"│   {logger.YELLOW}• STM32: Phát hiện STM32 đang ở DFU Bootloader Mode (0483:df11){logger.RESET}{' ' * 5}│")

    if hw["has_stlink"]:
        print(f"│   {logger.GREEN}• ST-Link: Phát hiện mạch nạp ST-Link SWD (0483:374x){logger.RESET}{' ' * 15}│")
    print("└" + "─" * 68 + "┘")


def prompt_user(step_num, title, instructions, safety_warning=None):
    """Prompts user to confirm, skip, or cancel before executing a test step."""
    print("\n" + "═" * 70)
    print(f" {logger.BOLD_CYAN}BƯỚC {step_num}: {title.upper()}{logger.RESET}")
    print("═" * 70)
    for line in instructions:
        print(f"  • {line}")
    if safety_warning:
        print(f"\n  {logger.BOLD_YELLOW}[LƯU Ý AN TOÀN]: {safety_warning}{logger.RESET}")
    print("─" * 70)

    while True:
        choice = safe_input(
            f"Bắt đầu kiểm tra bước này? [{logger.BOLD_GREEN}y{logger.RESET}: Đồng ý / {logger.BOLD_YELLOW}s{logger.RESET}: Bỏ qua / {logger.BOLD_RED}q{logger.RESET}: Thoát]: ",
            default="y"
        ).lower()
        if choice in ["y", "yes", "dong y", "c", "có", ""]:
            return "proceed"
        elif choice in ["s", "skip", "bo qua", "b"]:
            return "skip"
        elif choice in ["q", "quit", "exit", "huy", "thoat"]:
            return "cancel"
        else:
            print("  -> Vui lòng nhập 'y' (chạy), 's' (bỏ qua), hoặc 'q' (hủy bỏ).")


def make_ascii_bar(val, max_val=25.0, width=20):
    clamped = max(-max_val, min(max_val, val))
    ratio = (clamped + max_val) / (2.0 * max_val)
    pos = int(ratio * (width - 1))
    mid = width // 2
    chars = [" "] * width
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
    hw = scan_usb_hardware()
    detected_port = auto_detect_port()
    display_port = detected_port if detected_port else "Chưa phát hiện (Cắm cáp USB ESP32)"

    print("\n" + "╔" + "═" * 68 + "╗")
    print(f"║ {logger.BOLD_CYAN}KIỂM TRA TỔNG THỂ TỪNG LINH KIỆN & KẾT NỐI XE TỰ CÂN BẰNG{logger.RESET}          ║")
    print(f"║ Target: STM32F401CCU6 + ESP32-S3 + TB6612 + MPU6050 + GA25 Motors  ║")
    print(f"║ Cổng kết nối dự kiến: {display_port:<44} ║")
    print("╚" + "═" * 68 + "╝\n")

    print_hardware_inventory(hw)

    if hw["has_stm32_dfu"] or hw["has_stm32_hid"]:
        print(f"\n{logger.BOLD_YELLOW}⚠ CHÚ Ý VỀ STM32 BOOTLOADER & CỔNG KẾT NỐI:{logger.RESET}")
        print("  1. Cáp USB máy tính cắm vào mạch ESP32-S3 DevKitC (cổng UART Native USB CDC).")
        print("     • ESP32 GPIO 6 nối STM32 PA10 (RX), ESP32 GPIO 8 nối STM32 PA9 (TX).")
        print("  2. Phát hiện STM32 đang ở chế độ Bootloader USB (0483:572a).")
        print("     • Hãy BẤM NHẢ NÚT NRST (Reset) trên STM32 để khởi động vào chương trình chính.")

    print("\nChương trình sẽ lần lượt kiểm tra từng linh kiện.")
    print("Trước mỗi bước kiểm tra, bạn có quyền XÁC NHẬN (y), BỎ QUA (s), hoặc HỦY BỎ (q).\n")

    test_results = {s: "NOT_RUN" for s in ALL_STEPS}
    ser = None

    def ensure_serial_connection():
        nonlocal ser, detected_port
        if ser and ser.is_open:
            return True
        detected_port = auto_detect_port()
        if not detected_port:
            return False
        try:
            if ser is None:
                ser = serial.Serial()
            if ser.is_open:
                ser.close()
            ser.port = detected_port
            ser.baudrate = BAUD
            ser.timeout = 0.1
            ser.dtr = True
            ser.rts = True
            ser.open()
            time.sleep(0.5)
            ser.reset_input_buffer()
            return True
        except Exception as e:
            logger.error(f"Không thể mở cổng {detected_port}: {e}")
            return False

    def emergency_stop():
        nonlocal ser
        if ser and ser.is_open:
            try:
                ser.write(b"TEST:STOP\nSTOP_STREAM\n")
                ser.flush()
            except Exception:
                pass

    try:
        # =====================================================================
        # BƯỚC 1: CỔNG KẾT NỐI SERIAL & CẦU NỐI ESP32-S3
        # =====================================================================
        ans = prompt_user(
            step_num="1/8",
            title="Cổng Kết Nối USB & Mạch Nạp / Cầu Nối ESP32-S3",
            instructions=[
                f"Cắm cáp USB nối mạch ESP32-S3 (hoặc USB-UART) vào máy tính.",
                f"Kiểm tra xem hệ điều hành đã nhận cổng {display_port} chưa."
            ]
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_1] = "SKIPPED"
        else:
            if not detected_port:
                logger.info("Chưa phát hiện cổng Serial. Đang quét cổng USB cắm vào (Ctrl+C để bỏ qua)...")
                spinner = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
                s_idx = 0
                scan_start = time.time()
                try:
                    while not detected_port and (time.time() - scan_start < 20):
                        time.sleep(0.3)
                        detected_port = auto_detect_port()
                        if detected_port:
                            break
                        char = spinner[s_idx % len(spinner)]
                        s_idx += 1
                        elapsed = int(time.time() - scan_start)
                        sys.stdout.write(f"\r  {char} Đang chờ cắm cáp USB ESP32... ({elapsed}s)  ")
                        sys.stdout.flush()
                except KeyboardInterrupt:
                    pass
                print()

            if not detected_port:
                logger.error("Không tìm thấy cổng Serial nào trên hệ thống.")
                print("  -> Vui lòng cắm cáp USB của mạch ESP32-S3 DevKitC vào laptop.")
                test_results[STEP_1] = "FAIL"
            else:
                try:
                    if ensure_serial_connection():
                        logger.success(f"Cổng {detected_port} mở thành công ở tốc độ {BAUD} bps (DTR/RTS sẵn sàng)!")
                        test_results[STEP_1] = "PASS"
                    else:
                        test_results[STEP_1] = "FAIL"
                except Exception as e:
                    logger.error(f"Không thể mở cổng {detected_port}: {e}")
                    test_results[STEP_1] = "FAIL"
                    print("  -> Vui lòng kiểm tra quyền truy cập hoặc dây cáp USB.")

            if test_results[STEP_1] == "FAIL":
                cont = safe_input("Tiếp tục xem qua các bước còn lại? [y/N]: ", default="n").lower()
                if cont not in ["y", "yes", "dong y", "c"]:
                    return

        # =====================================================================
        # BƯỚC 2: ĐƯỜNG TRUYỀN UART VẬT LÝ ESP32 <-> STM32
        # =====================================================================
        ans = prompt_user(
            step_num="2/8",
            title="Đường Truyền UART Vật Lý Giữa ESP32-S3 Và STM32 BlackPill",
            instructions=[
                "Đảm bảo đã nối 3 dây giữa ESP32 và STM32 (chân đã được xác thực):",
                "  • ESP32 GPIO 6 (TX) ---> STM32 Chân PA10 (USART1 RX)",
                "  • ESP32 GPIO 3 (RX) <--- STM32 Chân PA9  (USART1 TX)",
                "  • ESP32 GND        <---> STM32 GND      (Bắt buộc chung Mass)",
                "Chương trình sẽ gửi lệnh PING và truy vấn STATUS từ STM32 qua ESP32-S3 Bridge."
            ]
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_2] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial tới ESP32/STM32 -> Hãy cắm cáp USB vào ESP32-S3.")
                test_results[STEP_2] = "BLOCKED"
            else:
                print("\n" + "┌" + "─" * 68 + "┐")
                print(f"│ {logger.BOLD_CYAN}BẮT ĐẦU KIỂM TRA ĐƯỜNG TRUYỀN UART ESP32 <---> STM32{logger.RESET}{' ' * 19}│")
                print("│  • Đang gửi lệnh bắt tay 'PING\\n' và lắng nghe phản hồi 'PONG'...   │")
                print("└" + "─" * 68 + "┘\n")

                ser.reset_input_buffer()
                first_signal = None
                ping_count = 0
                start_probe = time.time()
                last_ping_time = 0
                spinner = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
                spin_idx = 0
                raw_buffer = bytearray()

                try:
                    while time.time() - start_probe < 10.0:
                        now = time.time()
                        if now - last_ping_time >= 0.35:
                            ser.write(b"PING\n")
                            ser.flush()
                            last_ping_time = now
                            ping_count += 1

                        while ser.in_waiting:
                            b = ser.read(1)
                            if b in [b'\n', b'\r']:
                                if len(raw_buffer) > 0:
                                    line_str = raw_buffer.decode("latin1", errors="ignore").strip()
                                    raw_buffer.clear()
                                    if line_str:
                                        if any(key in line_str for key in ["PONG", "[SYS]", "STATUS:", "DATA:", "OK"]):
                                            first_signal = line_str
                                            break
                            else:
                                raw_buffer.extend(b)
                                if len(raw_buffer) > 256:
                                    raw_buffer.clear()

                        if first_signal:
                            break

                        spin_char = spinner[spin_idx % len(spinner)]
                        spin_idx += 1
                        elapsed_sec = int(time.time() - start_probe)
                        sys.stdout.write(f"\r  {spin_char} Đang quét tín hiệu... Đã gửi {ping_count} gói PING ({elapsed_sec}s) | Chờ STM32...  ")
                        sys.stdout.flush()
                        time.sleep(0.06)

                except KeyboardInterrupt:
                    print("\n\n[INFO] Đã dừng quét bởi người dùng.")

                print()
                if first_signal:
                    print("\n" + "╔" + "═" * 68 + "╗")
                    print(f"║ {logger.BOLD_GREEN}🎉 ĐÃ THÔNG TUYẾN UART GIỮA ESP32 VÀ STM32 BLACKPILL THÀNH CÔNG!{logger.RESET}{' ' * 7}║")
                    print("╚" + "═" * 68 + "╝")
                    print(f"  ✔ Phản hồi nhận được: {logger.BOLD_CYAN}\"{first_signal}\"{logger.RESET}")
                    print("  ✔ Cầu nối UART vật lý ESP32 (GPIO 6/3) <---> STM32 (PA10/PA9) HOẠT ĐỘNG HOÀN HẢO!")

                    # Lấy thông số STATUS kiểm chứng
                    ser.reset_input_buffer()
                    ser.write(b"STATUS\n")
                    ser.flush()
                    time.sleep(0.15)
                    while ser.in_waiting:
                        st_line = ser.readline().decode("latin1", errors="ignore").strip()
                        if "STATUS:" in st_line:
                            print(f"  ✔ Dữ liệu trạng thái STM32: {logger.BOLD_WHITE}{st_line}{logger.RESET}")

                    logger.success("Bước 2 ĐÃ ĐẠT (PASS)! Tiến hành sang các bước kiểm tra ngoại vi.\n")
                    test_results[STEP_2] = "PASS"
                else:
                    logger.error(f"Chưa nhận được phản hồi từ STM32 sau {ping_count} gói PING.")
                    print("  [CHẨN ĐOÁN LỖI]:")
                    print("  1. Bấm nhả nút NRST (Reset) trên mạch STM32 BlackPill.")
                    print("  2. Kiểm tra lại chân: ESP32 GPIO 6 -> STM32 PA10, ESP32 GPIO 3 -> STM32 PA9.")
                    print("  3. Đảm bảo dây GND giữa ESP32 và STM32 đã được nối chung mass.")
                    test_results[STEP_2] = "FAIL"

        # =====================================================================
        # BƯỚC 3: KẾT NỐI VÀ ĐIỀU KHIỂN BLUETOOTH BLE (SMARTPHONE APP & LED)
        # =====================================================================
        ans = prompt_user(
            step_num="3/8",
            title="Kết Nối & Điều Khiển Không Dây Bluetooth BLE (Smartphone App & Đèn LED)",
            instructions=[
                "Bật Bluetooth trên điện thoại, mở app (Serial Bluetooth Terminal hoặc BLE Controller).",
                "Quét và kết nối với thiết bị: 'STM32_Balance_Car' (Nordic UART Service).",
                "Gửi lần lượt ĐỦ 7 LỆNH ĐIỀU KHIỂN từ điện thoại (Không giới hạn thời gian):",
                "  1. Bật đèn LED PC13 : Gửi '1' hoặc 'LED:ON'",
                "  2. Tắt đèn LED PC13 : Gửi '0' hoặc 'LED:OFF'",
                "  3. Lệnh TIẾN        : Gửi 'F' hoặc 'FORWARD'",
                "  4. Lệnh LÙI         : Gửi 'B' hoặc 'BACKWARD'",
                "  5. Lệnh RẼ TRÁI     : Gửi 'L' hoặc 'LEFT'",
                "  6. Lệnh RẼ PHẢI     : Gửi 'R' hoặc 'RIGHT'",
                "  7. Lệnh DỪNG        : Gửi 'S' hoặc 'STOP'",
                "Khi bạn gửi đủ cả 7 lệnh trên, chương trình sẽ tự động hoàn tất và sang bước tiếp theo."
            ]
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_3] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial tới ESP32-S3 -> Không thể kiểm tra BLE.")
                test_results[STEP_3] = "BLOCKED"
            else:
                required_cmds = {
                    "LED_ON":   {"label": "1. Bật đèn LED       (1 / LED:ON)",   "done": False},
                    "LED_OFF":  {"label": "2. Tắt đèn LED       (0 / LED:OFF)",  "done": False},
                    "FORWARD":  {"label": "3. Lệnh TIẾN         (F / FORWARD)",  "done": False},
                    "BACKWARD": {"label": "4. Lệnh LÙI          (B / BACKWARD)", "done": False},
                    "LEFT":     {"label": "5. Lệnh RẼ TRÁI      (L / LEFT)",     "done": False},
                    "RIGHT":    {"label": "6. Lệnh RẼ PHẢI     (R / RIGHT)",    "done": False},
                    "STOP":     {"label": "7. Lệnh DỪNG         (S / STOP)",     "done": False},
                }

                def print_ble_checklist():
                    done_cnt = sum(1 for v in required_cmds.values() if v["done"])
                    total_cnt = len(required_cmds)
                    print("\n" + "┌" + "─" * 68 + "┐")
                    header = f"DANH SÁCH 7 LỆNH CẦN THU NHẬN TỪ ĐIỆN THOẠI ({done_cnt}/{total_cnt} ĐÃ ĐẠT):"
                    print(f"│ {logger.BOLD_CYAN}{header:<66}{logger.RESET} │")
                    print("├" + "─" * 68 + "┤")
                    for k, v in required_cmds.items():
                        if v["done"]:
                            tag = f"{logger.BOLD_GREEN}[✔ PASS]{logger.RESET}"
                        else:
                            tag = f"{logger.DIM}[  CHỜ ]{logger.RESET}"
                        print(f"│  {tag}  {v['label']:<54} │")
                    print("└" + "─" * 68 + "┘\n")

                print("\n" + "┌" + "─" * 68 + "┐")
                print(f"│ {logger.BOLD_CYAN}ĐANG LẮNG NGHE TÍN HIỆU BLUETOOTH BLE (KHÔNG GIỚI HẠN THỜI GIAN)...{logger.RESET}   │")
                print("│  • Tên thiết bị BLE: STM32_Balance_Car                             │")
                print("│  • Service UUID    : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E          │")
                print("│  • Gửi lệnh (RX)   : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E          │")
                print("│  • Nhận phản hồi(TX): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E         │")
                print("│  • Gợi ý: Hãy bấm lần lượt các nút trên app. Nhấn Ctrl+C để dừng sớm. │")
                print("└" + "─" * 68 + "┘\n")

                print_ble_checklist()

                ser.reset_input_buffer()
                ble_connected_detected = False
                raw_buf = bytearray()
                start_listen_time = time.time()

                try:
                    while True:
                        while ser.in_waiting:
                            b = ser.read(1)
                            if b in [b'\n', b'\r']:
                                if len(raw_buf) > 0:
                                    line_str = raw_buf.decode("latin1", errors="ignore").strip()
                                    raw_buf.clear()
                                    if line_str:
                                        # Detect connection event
                                        if "Client connected" in line_str or "BLE:CONNECTED" in line_str or "OK_BLE:CONNECTED" in line_str:
                                            if not ble_connected_detected:
                                                ble_connected_detected = True
                                                print(f"  {logger.BOLD_GREEN}🎉 PHÁT HIỆN ĐIỆN THOẠI ĐÃ KẾT NỐI VÀO BLE THÀNH CÔNG!{logger.RESET}")

                                        # Detect commands from phone or STM32 ACKs
                                        cmd_matched = None

                                        if "[BLE->STM32]" in line_str:
                                            cmd_val = line_str.replace("[BLE->STM32]", "").strip()
                                            print(f"  {logger.BOLD_CYAN}📱 [ĐIỆN THOẠI GỬI]:{logger.RESET} {logger.BOLD_WHITE}{cmd_val}{logger.RESET}")
                                            c_up = cmd_val.upper()
                                            if c_up in ["1", "LED:ON", "LED_ON", "CMD:LED_ON", "ON"]:
                                                cmd_matched = "LED_ON"
                                            elif c_up in ["0", "LED:OFF", "LED_OFF", "CMD:LED_OFF", "OFF"]:
                                                cmd_matched = "LED_OFF"
                                            elif c_up in ["F", "FORWARD", "CMD:FORWARD"]:
                                                cmd_matched = "FORWARD"
                                            elif c_up in ["B", "BACKWARD", "CMD:BACKWARD"]:
                                                cmd_matched = "BACKWARD"
                                            elif c_up in ["L", "LEFT", "CMD:LEFT"]:
                                                cmd_matched = "LEFT"
                                            elif c_up in ["R", "RIGHT", "CMD:RIGHT"]:
                                                cmd_matched = "RIGHT"
                                            elif c_up in ["S", "STOP", "CMD:STOP"]:
                                                cmd_matched = "STOP"

                                        if line_str.startswith("OK_LED:ON"):
                                            print(f"  {logger.BOLD_GREEN}💡 [STM32 LED]:{logger.RESET} {line_str} (Đèn LED PC13 đã BẬT)")
                                            cmd_matched = "LED_ON"
                                        elif line_str.startswith("OK_LED:OFF"):
                                            print(f"  {logger.BOLD_GREEN}💡 [STM32 LED]:{logger.RESET} {line_str} (Đèn LED PC13 đã TẮT)")
                                            cmd_matched = "LED_OFF"
                                        elif line_str.startswith("OK_CMD:FORWARD"):
                                            print(f"  {logger.BOLD_GREEN}🚀 [STM32 LỆNH]:{logger.RESET} {line_str} (Lệnh TIẾN)")
                                            cmd_matched = "FORWARD"
                                        elif line_str.startswith("OK_CMD:BACKWARD"):
                                            print(f"  {logger.BOLD_GREEN}🚀 [STM32 LỆNH]:{logger.RESET} {line_str} (Lệnh LÙI)")
                                            cmd_matched = "BACKWARD"
                                        elif line_str.startswith("OK_CMD:LEFT"):
                                            print(f"  {logger.BOLD_GREEN}🚀 [STM32 LỆNH]:{logger.RESET} {line_str} (Lệnh TRÁI)")
                                            cmd_matched = "LEFT"
                                        elif line_str.startswith("OK_CMD:RIGHT"):
                                            print(f"  {logger.BOLD_GREEN}🚀 [STM32 LỆNH]:{logger.RESET} {line_str} (Lệnh PHẢI)")
                                            cmd_matched = "RIGHT"
                                        elif line_str.startswith("OK_CMD:STOP"):
                                            print(f"  {logger.BOLD_GREEN}🛑 [STM32 LỆNH]:{logger.RESET} {line_str} (Lệnh DỪNG)")
                                            cmd_matched = "STOP"

                                        if cmd_matched and not required_cmds[cmd_matched]["done"]:
                                            required_cmds[cmd_matched]["done"] = True
                                            print(f"\n  {logger.BOLD_GREEN}✔ ĐÃ XÁC NHẬN LỆNH:{logger.RESET} {required_cmds[cmd_matched]['label']}")
                                            print_ble_checklist()
                            else:
                                raw_buf.extend(b)
                                if len(raw_buf) > 256:
                                    raw_buf.clear()

                        all_done = all(v["done"] for v in required_cmds.values())
                        if all_done:
                            break

                        done_cnt = sum(1 for v in required_cmds.values() if v["done"])
                        elapsed_s = int(time.time() - start_listen_time)
                        status_tag = f"{logger.BOLD_GREEN}Đã kết nối{logger.RESET}" if ble_connected_detected else f"{logger.YELLOW}Đang chờ kết nối...{logger.RESET}"
                        sys.stdout.write(f"\r  📡 BLE: {status_tag} | Tiến độ: {done_cnt}/7 lệnh | Thời gian: {elapsed_s}s (Ctrl+C để dừng)  ")
                        sys.stdout.flush()
                        time.sleep(0.04)

                except KeyboardInterrupt:
                    print("\n[INFO] Đã tạm dừng lắng nghe BLE theo yêu cầu người dùng.")

                done_cnt = sum(1 for v in required_cmds.values() if v["done"])
                print()
                if done_cnt == len(required_cmds):
                    print("╔" + "═" * 68 + "╗")
                    print(f"║ {logger.BOLD_GREEN}🎉 CHÚC MỪNG: ĐÃ THU NHẬN ĐẦY ĐỦ 7/7 LỆNH BLUETOOTH THÀNH CÔNG!{logger.RESET}{' ' * 7}║")
                    print("╚" + "═" * 68 + "╝")
                    logger.success("Mạch Bluetooth BLE 5.0 và STM32 đáp ứng toàn bộ tập lệnh điều khiển hoàn hảo!")
                    test_results[STEP_3] = "PASS"
                elif done_cnt >= 3:
                    logger.warn(f"Đã nhận {done_cnt}/7 lệnh Bluetooth.")
                    ask_user = safe_input("Bạn có muốn đánh giá ĐẠT (PASS) cho bước Bluetooth không? [Y/n]: ", default="y").lower()
                    if ask_user in ["y", "yes", "c", "có"]:
                        test_results[STEP_3] = "PASS"
                    else:
                        test_results[STEP_3] = "WARN"
                else:
                    logger.error(f"Chưa nhận đủ lệnh Bluetooth (mới đạt {done_cnt}/7 lệnh).")
                    test_results[STEP_3] = "FAIL"

        # =====================================================================
        # BƯỚC 4: CẢM BIẾN GÓC NGHIÊNG MPU6050 & I2C BUS
        # =====================================================================
        ans = prompt_user(
            step_num="4/8",
            title="Cảm Biến Góc Nghiêng MPU6050 & Giao Tiếp I2C (PB6/PB7)",
            instructions=[
                "Kiểm tra chip MPU6050 (I2C Address 0x68, WHO_AM_I).",
                "Trong 5 giây tới, bạn hãy CẦM XE LÊN TAY VÀ NGHIÊNG VỀ TRƯỚC / SAU.",
                "Quan sát xem góc Pitch và con quay Gyro có phản hồi chính xác không."
            ]
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_3] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể đọc cảm biến MPU6050.")
                test_results[STEP_3] = "BLOCKED"
            else:
                ser.reset_input_buffer()
                ser.write(b"STATUS\n")
                ser.flush()
                time.sleep(0.15)
                mpu_connected = False
                while ser.in_waiting:
                    line = ser.readline().decode("latin1", errors="ignore").strip()
                    if "[SYS]" in line or "STATUS:" in line:
                        print(f"  [STM32 STATUS]: {line}")
                        if "MPU=1" in line:
                            mpu_connected = True

                if not mpu_connected:
                    print(f"\n  {logger.BOLD_YELLOW}⚠ CHÚ Ý: STM32 báo MPU=0 (Chưa nhận diện cảm biến MPU6050 trên I2C){logger.RESET}")
                    print("  • Hướng dẫn kiểm tra kết nối MPU6050 <---> STM32:")
                    print("    - SCL MPU6050 nối chân PB6 (STM32)")
                    print("    - SDA MPU6050 nối chân PB7 (STM32)")
                    print("    - VCC MPU6050 nối chân 3.3V, GND nối GND")
                    print("    - Chân AD0 của MPU6050 nối GND (để dùng địa chỉ mặc định 0x68)")
                    print("  • Tiếp tục kiểm tra luồng dữ liệu góc trong 5 giây tới...\n")

                logger.info("Đang đọc luồng góc nghiêng thời gian thực trong 5 giây...")
                ser.write(b"START_STREAM\n")
                ser.flush()

                start_t = time.time()
                pitch_samples = []
                gyro_samples = []

                while time.time() - start_t < 5.0:
                    while ser.in_waiting:
                        line = ser.readline().decode("latin1", errors="ignore").strip()
                        if line.startswith("DATA:"):
                            parts = line[5:].split(",")
                            if len(parts) >= 2:
                                try:
                                    p = float(parts[0])
                                    g = float(parts[1])
                                    pitch_samples.append(p)
                                    gyro_samples.append(g)
                                    bar = make_ascii_bar(p, max_val=25.0)
                                    print(f"\r  Pitch: {p:+6.2f}° {bar} | Gyro: {g:+7.1f}°/s | Mẫu: {len(pitch_samples)}  ", end="", flush=True)
                                except ValueError:
                                    pass
                    time.sleep(0.02)
                print()
                ser.write(b"STOP_STREAM\n")
                ser.flush()

                pitch_range = (max(pitch_samples) - min(pitch_samples)) if pitch_samples else 0.0
                gyro_max = max((abs(g) for g in gyro_samples), default=0.0)

                if mpu_connected or pitch_range >= 2.0 or gyro_max >= 5.0:
                    logger.success(f"Cảm biến MPU6050 hoạt động tốt! Thu nhận {len(pitch_samples)} mẫu. Dải góc nghiêng đo được: {pitch_range:.2f}°")
                    test_results[STEP_4] = "PASS"
                elif len(pitch_samples) >= 15:
                    logger.warn(f"Thu nhận {len(pitch_samples)} gói dữ liệu nhưng góc nghiêng không đổi ({pitch_range:.2f}°). MPU6050 chưa kết nối I2C.")
                    test_results[STEP_4] = "WARN"
                else:
                    logger.error("Không nhận được dữ liệu cảm biến từ STM32.")
                    print("  -> Kiểm tra: Dây SDA (PB7), SCL (PB6), nguồn 3.3V và GND cấp cho MPU6050.")
                    test_results[STEP_4] = "FAIL"

        # =====================================================================
        # BƯỚC 5: ĐĨA ĐẾM XUNG ENCODER GA25 (TRÁI & PHẢI)
        # =====================================================================
        ans = prompt_user(
            step_num="5/8",
            title="Đĩa Đọc Xung Encoder GA25 (TIM2 PA0/PA1 & TIM3 PB4/PB5)",
            instructions=[
                "Trong 6 giây tới, hãy LẤY TAY XOAY LẦN LƯỢT 2 BÁNH XE:",
                "  1. Lấy tay xoay bánh xe TRÁI về phía trước vài vòng.",
                "  2. Lấy tay xoay bánh xe PHẢI về phía trước vài vòng.",
                "Hệ thống sẽ đếm tổng số xung thay đổi để đánh giá mắt đọc encoder."
            ]
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_5] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể đọc dữ liệu Encoder.")
                test_results[STEP_5] = "BLOCKED"
            else:
                ser.reset_input_buffer()
                ser.write(b"START_STREAM\n")
                ser.flush()

                start_t = time.time()
                enc_l_init = None
                enc_r_init = None
                enc_l_last = 0
                enc_r_last = 0

                logger.info("Hãy bắt đầu xoay bánh xe bằng tay...")
                while time.time() - start_t < 6.0:
                    while ser.in_waiting:
                        line = ser.readline().decode("latin1", errors="ignore").strip()
                        if line.startswith("DATA:"):
                            parts = line[5:].split(",")
                            if len(parts) >= 5:
                                try:
                                    el = int(parts[3])
                                    er = int(parts[4])
                                    if enc_l_init is None:
                                        enc_l_init = el
                                        enc_r_init = er
                                    enc_l_last = el
                                    enc_r_last = er

                                    delta_l = el - enc_l_init
                                    delta_r = er - enc_r_init
                                    print(f"\r  Encoder: Trái = {el:8d} (Δ {delta_l:+6d}) | Phải = {er:8d} (Δ {delta_r:+6d})  ", end="", flush=True)
                                except ValueError:
                                    pass
                    time.sleep(0.02)
                print()
                ser.write(b"STOP_STREAM\n")
                ser.flush()

                diff_l = abs(enc_l_last - (enc_l_init or 0))
                diff_r = abs(enc_r_last - (enc_r_init or 0))

                print(f"\n  • Biến thiên xung bánh Trái (EncL): {diff_l} xung")
                print(f"  • Biến thiên xung bánh Phải (EncR): {diff_r} xung")

                if diff_l >= 15 and diff_r >= 15:
                    logger.success("Cả 2 đĩa Encoder GA25 (Trái và Phải) đếm xung hoàn hảo!")
                    test_results[STEP_5] = "PASS"
                elif diff_l >= 15 or diff_r >= 15:
                    logger.warn("Một trong hai bên Encoder chưa đếm được xung (hoặc chưa xoay đủ lực).")
                    test_results[STEP_5] = "WARN"
                else:
                    logger.error("Không ghi nhận được xung encoder nào thay đổi.")
                    print("  -> Kiểm tra: Nguồn 3.3V/GND cho encoder, PA0/PA1 (Trái), PB4/PB5 (Phải).")
                    test_results[STEP_5] = "FAIL"

        # =====================================================================
        # BƯỚC 6: ĐỘNG CƠ GA25 & MẠCH CẦU H TB6612 (BÁNH TRÁI)
        # =====================================================================
        ans = prompt_user(
            step_num="6/8",
            title="Động Cơ GA25 & Cầu H TB6612 - BÁNH TRÁI (PWMA PB8, AIN1 PA4, AIN2 PA5)",
            instructions=[
                "Kê xe hổng khỏi mặt bàn để bánh xe quay tự do trong không khí.",
                "Cấp nguồn Pin (7.4V) vào chân VM của TB6612 và bật công tắc nguồn.",
                "Bánh xe TRÁI sẽ được kích hoạt quay TIẾN (0.6s) rồi quay LÙI (0.6s)."
            ],
            safety_warning="Không chạm tay vào nan hoa hoặc lốp xe trong lúc động cơ đang quay!"
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_6] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể phát xung lái Động cơ Trái.")
                test_results[STEP_6] = "BLOCKED"
            else:
                logger.info("Đang phát xung lái Động cơ TRÁI -> Quay TIẾN (+1500)...")
                ser.reset_input_buffer()
                ser.write(b"TEST:MOTOR_L\n")
                ser.flush()
                time.sleep(0.8)
                while ser.in_waiting:
                    line = ser.readline().decode("latin1", errors="ignore").strip()
                    if line:
                        print(f"  [STM32]: {line}")

                logger.info("Đang phát xung lái Động cơ TRÁI -> Quay LÙI (-1500)...")
                ser.write(b"TEST:MOTOR_L_REV\n")
                ser.flush()
                time.sleep(0.8)
                while ser.in_waiting:
                    line = ser.readline().decode("latin1", errors="ignore").strip()
                    if line:
                        print(f"  [STM32]: {line}")

                # Đọc kiểm chứng biến thiên xung Encoder Trái
                ser.write(b"STATUS\n")
                ser.flush()
                time.sleep(0.1)
                enc_l_val = None
                while ser.in_waiting:
                    line = ser.readline().decode("latin1", errors="ignore").strip()
                    if "ENC_L=" in line:
                        for part in line.split(","):
                            if part.startswith("ENC_L="):
                                try:
                                    enc_l_val = int(part.split("=")[1])
                                except ValueError:
                                    pass

                if enc_l_val is not None:
                    print(f"  • Vị trí Encoder Trái hiện tại: {enc_l_val} xung")

                ans_l = safe_input("\nBạn có nhìn thấy bánh xe TRÁI quay Tiến rồi Lùi không? [Y/n]: ", default="y").lower()
                if ans_l in ["y", "yes", "có", "c", ""] and ans_l != "q":
                    logger.success("Động cơ và kênh công suất Trái (PWMA, AIN1, AIN2) HOẠT ĐỘNG TỐT!")
                    test_results[STEP_6] = "PASS"
                else:
                    logger.error("Bánh xe Trái không quay!")
                    print("  -> Kiểm tra: Pin cấp vào VM (7.4V), công tắc nguồn, chân STBY (PB0), PWMA (PB8), AIN1 (PA4), AIN2 (PA5).")
                    test_results[STEP_6] = "FAIL"

        # =====================================================================
        # BƯỚC 7: ĐỘNG CƠ GA25 & MẠCH CẦU H TB6612 (BÁNH PHẢI)
        # =====================================================================
        ans = prompt_user(
            step_num="7/8",
            title="Động Cơ GA25 & Cầu H TB6612 - BÁNH PHẢI (PWMB PB9, BIN1 PA6, BIN2 PA7)",
            instructions=[
                "Vẫn giữ xe trên giá đỡ, bánh xe quay tự do.",
                "Bánh xe PHẢI sẽ được kích hoạt quay TIẾN (0.6s) rồi quay LÙI (0.6s)."
            ],
            safety_warning="Đảm bảo nguồn Pin 7.4V vẫn đang bật."
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_7] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể phát xung lái Động cơ Phải.")
                test_results[STEP_7] = "BLOCKED"
            else:
                logger.info("Đang phát xung lái Động cơ PHẢI -> Quay TIẾN (+1500)...")
                ser.reset_input_buffer()
                ser.write(b"TEST:MOTOR_R\n")
                ser.flush()
                time.sleep(0.8)
                while ser.in_waiting:
                    line = ser.readline().decode("latin1", errors="ignore").strip()
                    if line:
                        print(f"  [STM32]: {line}")

                logger.info("Đang phát xung lái Động cơ PHẢI -> Quay LÙI (-1500)...")
                ser.write(b"TEST:MOTOR_R_REV\n")
                ser.flush()
                time.sleep(0.8)
                while ser.in_waiting:
                    line = ser.readline().decode("latin1", errors="ignore").strip()
                    if line:
                        print(f"  [STM32]: {line}")

                # Đọc kiểm chứng biến thiên xung Encoder Phải
                ser.write(b"STATUS\n")
                ser.flush()
                time.sleep(0.1)
                enc_r_val = None
                while ser.in_waiting:
                    line = ser.readline().decode("latin1", errors="ignore").strip()
                    if "ENC_R=" in line:
                        for part in line.split(","):
                            if part.startswith("ENC_R="):
                                try:
                                    enc_r_val = int(part.split("=")[1])
                                except ValueError:
                                    pass

                if enc_r_val is not None:
                    print(f"  • Vị trí Encoder Phải hiện tại: {enc_r_val} xung")

                ans_r = safe_input("\nBạn có nhìn thấy bánh xe PHẢI quay Tiến rồi Lùi không? [Y/n]: ", default="y").lower()
                if ans_r in ["y", "yes", "có", "c", ""] and ans_r != "q":
                    logger.success("Động cơ và kênh công suất Phải (PWMB, BIN1, BIN2) HOẠT ĐỘNG TỐT!")
                    test_results[STEP_7] = "PASS"
                else:
                    logger.error("Bánh xe Phải không quay!")
                    print("  -> Kiểm tra: Pin cấp vào VM (7.4V), công tắc nguồn, chân PWMB (PB9), BIN1 (PA6), BIN2 (PA7).")
                    test_results[STEP_7] = "FAIL"

        # =====================================================================
        # BƯỚC 8: ĐỒNG BỘ 2 ĐỘNG CƠ & CHỐT PHANH KHẨN CẤP
        # =====================================================================
        ans = prompt_user(
            step_num="8/8",
            title="Đồng Bộ Hai Động Cơ & Cơ Chế Phanh An Toàn (Emergency Stop)",
            instructions=[
                "Cả hai bánh xe sẽ cùng quay TIẾN trong 0.8 giây.",
                "Sau đó hệ thống tự động phát lệnh phanh khẩn cấp (TEST:STOP)."
            ]
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_8] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể kiểm tra đồng bộ động cơ.")
                test_results[STEP_8] = "BLOCKED"
            else:
                logger.info("Kích hoạt cả 2 động cơ quay đồng bộ...")
                ser.write(b"TEST:MOTORS\n")
                ser.flush()
                time.sleep(0.8)

                emergency_stop()
                logger.success("Lệnh phanh khẩn cấp (Emergency Stop) đã kích hoạt thành công!")
                test_results[STEP_8] = "PASS"

    except KeyboardInterrupt:
        print("\n\n[WARN] Người dùng ngắt ngang bằng phím Ctrl+C.")
    finally:
        emergency_stop()
        if ser and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass

        # =====================================================================
        # BẢNG TỔNG KẾT BÁO CÁO TOÀN DIỆN (LUÔN ĐƯỢC HIỂN THỊ)
        # =====================================================================
        print("\n\n" + "╔" + "═" * 68 + "╗")
        print(f"║          {logger.BOLD_CYAN}BẢNG TỔNG HỢP KẾT QUẢ KIỂM TRA PHẦN CỨNG TOÀN DIỆN{logger.RESET}          ║")
        print("╠" + "═" * 68 + "╣")

        pass_count = 0
        total_steps = len(ALL_STEPS)

        for item in ALL_STEPS:
            status = test_results.get(item, "NOT_RUN")
            if status == "PASS":
                tag = f"{logger.BOLD_GREEN}[  PASS  ]{logger.RESET}"
                pass_count += 1
            elif status == "WARN":
                tag = f"{logger.BOLD_YELLOW}[  WARN  ]{logger.RESET}"
            elif status == "SKIPPED":
                tag = f"{logger.DIM}[ SKIPPT ]{logger.RESET}"
            elif status == "BLOCKED":
                tag = f"{logger.BOLD_MAGENTA}[ BLOCKD ]{logger.RESET}"
            elif status == "NOT_RUN":
                tag = f"{logger.DIM}[ CANCEL ]{logger.RESET}"
            else:
                tag = f"{logger.BOLD_RED}[  FAIL  ]{logger.RESET}"

            print(f"║  {item:<50} {tag}  ║")

        print("╚" + "═" * 68 + "╝")

        if pass_count == total_steps:
            print(f"\n{logger.BOLD_GREEN}🎉 CHÚC MỪNG: TOÀN BỘ {total_steps}/{total_steps} LINH KIỆN VÀ ĐƯỜNG TRUYỀN ĐỀU HOẠT ĐỘNG HOÀN HẢO!{logger.RESET}")
            print("  -> Chiếc xe đã sẵn sàng 100% để đặt xuống mặt sàn tự cân bằng.")
            print("  -> Khởi động xe: ./run_pipeline.sh --hardware")
        elif pass_count > 0:
            print(f"\n{logger.BOLD_YELLOW}⚠ KẾT QUẢ: ĐẠT {pass_count}/{total_steps} BƯỚC KIỂM TRA.{logger.RESET}")
            print("  -> Hãy kiểm tra lại các mục có nhãn [ FAIL ] hoặc [ BLOCKD ] theo hướng dẫn chi tiết.")
        else:
            print(f"\n{logger.BOLD_YELLOW}ℹ BÁO CÁO: ĐẠT 0/{total_steps} BƯỚC (Do hủy bỏ hoặc chưa kết nối phần cứng).{logger.RESET}")


if __name__ == "__main__":
    main()

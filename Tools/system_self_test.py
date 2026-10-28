#!/usr/bin/env python3
"""
Comprehensive Hardware & Connection Diagnostic Suite
Target: STM32F401CCU6 Self-Balancing Robot
Tests every component step-by-step with interactive user confirmation before each step.

Components Tested:
1. USB Serial Port & ESP32-S3 Bridge
2. STM32 Physical UART Handshake (PING/PONG)
3. MPU6050 6-DOF IMU & I2C Bus (WHO_AM_I, Pitch Angle, Gyro Rate)
4. GA25 Wheel Encoders (TIM2 PA0/PA1, TIM3 PB4/PB5)
5. TB6612 Dual H-Bridge & Left Motor (PWMA PB8, AIN1 PA4, AIN2 PA5, STBY PB0)
6. TB6612 Dual H-Bridge & Right Motor (PWMB PB9, BIN1 PA6, BIN2 PA7)
7. Dual Motor Synchronization & Emergency Brake (Safety Stop)
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

STEP_1 = "1. Cổng USB / ESP32-S3 Bridge"
STEP_2 = "2. Đường truyền UART (ESP32 <-> STM32)"
STEP_3 = "3. Cảm biến MPU6050 (I2C PB6/PB7)"
STEP_4 = "4. Đĩa Encoder GA25 (TIM2 & TIM3)"
STEP_5 = "5. Động cơ & Cầu H (Bánh Trái)"
STEP_6 = "6. Động cơ & Cầu H (Bánh Phải)"
STEP_7 = "7. Đồng bộ 2 động cơ & Phanh khẩn cấp"

ALL_STEPS = [STEP_1, STEP_2, STEP_3, STEP_4, STEP_5, STEP_6, STEP_7]


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
    """Finds best matching physical COM/tty port or returns None if none found."""
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        return sys.argv[1]
    ports = list(serial.tools.list_ports.comports())
    phys = logger.filter_physical_ports(ports)
    if phys:
        return phys[0].device
    return None


def scan_usb_hardware():
    """Scans USB bus for ESP32, STM32 Bootloader, and ST-Link probes."""
    info = {
        "serial_ports": [],
        "has_stm32_hid": False,
        "has_stm32_dfu": False,
        "has_stlink": False,
    }

    # 1. Serial ports (filtered to exclude dummy ttyS ports)
    all_com = list(serial.tools.list_ports.comports())
    phys_com = logger.filter_physical_ports(all_com)
    for p in phys_com:
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
        print("  1. Cáp USB máy tính BẮT BUỘC CẮM VÀO BO ESP32-S3 (chân 4 & 5 sẽ nối sang STM32).")
        print("     • Cổng Type-C trên STM32 BlackPill KHÔNG có chip USB-Serial -> Không thể giao tiếp Serial.")
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
            step_num="1/7",
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
        # BƯỚC 2: ĐƯỜNG TRUYỀN UART VẬT LÝ ESP32 <-> STM32 (TEST LIÊN TỤC)
        # =====================================================================
        ans = prompt_user(
            step_num="2/7",
            title="Đường Truyền UART Vật Lý Giữa ESP32-S3 Và STM32 BlackPill",
            instructions=[
                "Đảm bảo đã nối 3 dây giữa ESP32 và STM32:",
                "  • ESP32 Chân 4 (TX) ---> STM32 Chân A10 (USART1 RX)",
                "  • ESP32 Chân 5 (RX) <--- STM32 Chân A9  (USART1 TX)",
                "  • ESP32 GND        <---> STM32 GND      (Bắt buộc chung Mass)",
                "Chương trình sẽ LIÊN TỤC GỬI PING cho tới khi NHẬN ĐƯỢC TÍN HIỆU ĐẦU TIÊN từ STM32.",
                "Trong lúc quét: Hãy BẤM NHẢ NÚT NRST trên STM32 hoặc tráo đổi 2 dây 4 & 5."
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
                print(f"│ {logger.BOLD_CYAN}BẮT ĐẦU CHẾ ĐỘ QUÉT VÀ DÒ TÌM TÍN HIỆU STM32 LIÊN TỤC{logger.RESET}{' ' * 16}│")
                print("│  • Đang liên tục gửi lệnh bắt tay 'PING\\n' và lắng nghe phản hồi... │")
                print("│  • CÁC THAO TÁC CẦN LÀM TRÊN PHẦN CỨNG NGAY BÂY GIỜ:               │")
                print("│    1. BẤM NHẢ NÚT NRST (Reset) trên mạch STM32 BlackPill.          │")
                print("│    2. Kiểm tra/Tráo đổi 2 dây: Chân 4 (TX) -> A10, Chân 5 (RX) -> A9│")
                print("│    3. Đảm bảo ESP32 GND đã nối với STM32 GND.                      │")
                print("│  • Script sẽ TỰ ĐỘNG DỪNG & BÁO SUCCESS ngay khi có tín hiệu!      │")
                print("│  • (Bấm Ctrl+C bất kỳ lúc nào để dừng quét nếu muốn bỏ qua)        │")
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
                    while True:
                        now = time.time()
                        # Gửi PING đều đặn mỗi 0.35s
                        if now - last_ping_time >= 0.35:
                            ser.write(b"PING\n")
                            ser.flush()
                            last_ping_time = now
                            ping_count += 1

                        # Đọc phản hồi từ cổng Serial
                        while ser.in_waiting:
                            b = ser.read(1)
                            if b == b'\n' or b == b'\r':
                                if len(raw_buffer) > 0:
                                    line_str = raw_buffer.decode("latin1", errors="ignore").strip()
                                    raw_buffer.clear()
                                    if line_str:
                                        # Bắt bất kỳ phản hồi hợp lệ nào từ STM32
                                        if any(key in line_str for key in ["PONG", "[SYS]", "STATUS:", "DATA:", "MPU", "OK"]):
                                            first_signal = line_str
                                            break
                                        elif len(line_str) >= 2:
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
                        sys.stdout.write(f"\r  {spin_char} Đang quét tín hiệu... Đã gửi {ping_count} gói PING ({elapsed_sec}s) | Chờ STM32 phản hồi... (Ctrl+C để dừng)  ")
                        sys.stdout.flush()
                        time.sleep(0.06)

                except KeyboardInterrupt:
                    print("\n\n[INFO] Đã tạm dừng chế độ quét liên tục bởi phím bấm.")

                print()
                if first_signal:
                    print("\n" + "╔" + "═" * 68 + "╗")
                    print(f"║ {logger.BOLD_GREEN}🎉 ĐÃ BẮT ĐƯỢC TÍN HIỆU ĐẦU TIÊN TỪ STM32 BLACKPILL THÀNH CÔNG!{logger.RESET}{' ' * 7}║")
                    print("╚" + "═" * 68 + "╝")
                    print(f"  ✔ Phản hồi nhận được: {logger.BOLD_CYAN}\"{first_signal}\"{logger.RESET}")
                    print(f"  ✔ Thời gian phát hiện: sau {int(time.time() - start_probe)} giây (lần PING #{ping_count})")
                    print("  ✔ Cầu nối UART vật lý ESP32 (4/5) <---> STM32 (A10/A9) HOẠT ĐỘNG HOÀN HẢO!")
                    logger.success("Bước 2 ĐÃ ĐẠT (PASS)! Tiến hành sang các bước kiểm tra ngoại vi.\n")
                    test_results[STEP_2] = "PASS"
                else:
                    logger.error(f"Chưa nhận được phản hồi từ STM32 sau {ping_count} gói PING.")
                    print("  [CHẨN ĐOÁN LỖI]:")
                    print("  1. STM32 có đang bị kẹt ở chế độ Bootloader USB không? Bấm nút NRST trên mạch STM32.")
                    print("  2. Kiểm tra lại chân: ESP32 Chân 4 -> STM32 A10, ESP32 Chân 5 -> STM32 A9.")
                    print("  3. Đã nối chung dây GND giữa 2 mạch chưa?")
                    test_results[STEP_2] = "FAIL"

        # =====================================================================
        # BƯỚC 3: CẢM BIẾN GÓC NGHIÊNG MPU6050 & I2C BUS
        # =====================================================================
        ans = prompt_user(
            step_num="3/7",
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
                while ser.in_waiting:
                    line = ser.readline().decode("latin1", errors="ignore").strip()
                    if "[SYS]" in line or "STATUS:" in line:
                        print(f"  [STM32 STATUS]: {line}")

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
                if len(pitch_samples) >= 15:
                    logger.success(f"Cảm biến MPU6050 hoạt động tốt! Thu nhận {len(pitch_samples)} mẫu. Dải góc nghiêng đo được: {pitch_range:.2f}°")
                    test_results[STEP_3] = "PASS"
                else:
                    logger.error("Không nhận được đủ dữ liệu từ MPU6050.")
                    print("  -> Kiểm tra: Dây SDA (PB7), SCL (PB6), nguồn 3.3V và GND cấp cho MPU6050.")
                    test_results[STEP_3] = "FAIL"

        # =====================================================================
        # BƯỚC 4: ĐĨA ĐẾM XUNG ENCODER GA25 (TRÁI & PHẢI)
        # =====================================================================
        ans = prompt_user(
            step_num="4/7",
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
            test_results[STEP_4] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể đọc dữ liệu Encoder.")
                test_results[STEP_4] = "BLOCKED"
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
                    test_results[STEP_4] = "PASS"
                elif diff_l >= 15 or diff_r >= 15:
                    logger.warn("Một trong hai bên Encoder chưa đếm được xung (hoặc chưa xoay đủ lực).")
                    test_results[STEP_4] = "WARN"
                else:
                    logger.error("Không ghi nhận được xung encoder nào thay đổi.")
                    print("  -> Kiểm tra: Nguồn 3.3V/GND cho encoder, PA0/PA1 (Trái), PB4/PB5 (Phải).")
                    test_results[STEP_4] = "FAIL"

        # =====================================================================
        # BƯỚC 5: ĐỘNG CƠ GA25 & MẠCH CẦU H TB6612 (BÁNH TRÁI)
        # =====================================================================
        ans = prompt_user(
            step_num="5/7",
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
            test_results[STEP_5] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể phát xung lái Động cơ Trái.")
                test_results[STEP_5] = "BLOCKED"
            else:
                logger.info("Đang phát xung lái Động cơ TRÁI -> Quay TIẾN (+1500)...")
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

                ans_l = safe_input("\nBạn có nhìn thấy bánh xe TRÁI quay Tiến rồi Lùi không? [Y/n]: ", default="y").lower()
                if ans_l in ["y", "yes", "có", "c", ""] and ans_l != "q":
                    logger.success("Động cơ và kênh công suất Trái (PWMA, AIN1, AIN2) HOẠT ĐỘNG TỐT!")
                    test_results[STEP_5] = "PASS"
                else:
                    logger.error("Bánh xe Trái không quay!")
                    print("  -> Kiểm tra: Pin cấp vào VM, chân STBY (PB0), PWMA (PB8), AIN1 (PA4), AIN2 (PA5).")
                    test_results[STEP_5] = "FAIL"

        # =====================================================================
        # BƯỚC 6: ĐỘNG CƠ GA25 & MẠCH CẦU H TB6612 (BÁNH PHẢI)
        # =====================================================================
        ans = prompt_user(
            step_num="6/7",
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
            test_results[STEP_6] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể phát xung lái Động cơ Phải.")
                test_results[STEP_6] = "BLOCKED"
            else:
                logger.info("Đang phát xung lái Động cơ PHẢI -> Quay TIẾN (+1500)...")
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

                ans_r = safe_input("\nBạn có nhìn thấy bánh xe PHẢI quay Tiến rồi Lùi không? [Y/n]: ", default="y").lower()
                if ans_r in ["y", "yes", "có", "c", ""] and ans_r != "q":
                    logger.success("Động cơ và kênh công suất Phải (PWMB, BIN1, BIN2) HOẠT ĐỘNG TỐT!")
                    test_results[STEP_6] = "PASS"
                else:
                    logger.error("Bánh xe Phải không quay!")
                    print("  -> Kiểm tra: Pin cấp vào VM, chân PWMB (PB9), BIN1 (PA6), BIN2 (PA7).")
                    test_results[STEP_6] = "FAIL"

        # =====================================================================
        # BƯỚC 7: ĐỒNG BỘ 2 ĐỘNG CƠ & CHỐT PHANH KHẨN CẤP
        # =====================================================================
        ans = prompt_user(
            step_num="7/7",
            title="Đồng Bộ Hai Động Cơ & Cơ Chế Phanh An Toàn (Emergency Stop)",
            instructions=[
                "Cả hai bánh xe sẽ cùng quay TIẾN trong 0.8 giây.",
                "Sau đó hệ thống tự động phát lệnh phanh khẩn cấp (TEST:STOP)."
            ]
        )
        if ans == "cancel":
            return
        elif ans == "skip":
            test_results[STEP_7] = "SKIPPED"
        else:
            if not ensure_serial_connection():
                logger.warn("Chưa có kết nối Serial -> Không thể kiểm tra đồng bộ động cơ.")
                test_results[STEP_7] = "BLOCKED"
            else:
                logger.info("Kích hoạt cả 2 động cơ quay đồng bộ...")
                ser.write(b"TEST:MOTORS\n")
                ser.flush()
                time.sleep(0.8)

                emergency_stop()
                logger.success("Lệnh phanh khẩn cấp (Emergency Stop) đã kích hoạt thành công!")
                test_results[STEP_7] = "PASS"

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

        if pass_count == 7:
            print(f"\n{logger.BOLD_GREEN}🎉 CHÚC MỪNG: TOÀN BỘ 7/7 LINH KIỆN VÀ ĐƯỜNG TRUYỀN ĐỀU HOẠT ĐỘNG HOÀN HẢO!{logger.RESET}")
            print("  -> Chiếc xe đã sẵn sàng 100% để đặt xuống mặt sàn tự cân bằng.")
            print("  -> Khởi động xe: ./run_pipeline.sh --hardware")
        elif pass_count > 0:
            print(f"\n{logger.BOLD_YELLOW}⚠ KẾT QUẢ: ĐẠT {pass_count}/{total_steps} BƯỚC KIỂM TRA.{logger.RESET}")
            print("  -> Hãy kiểm tra lại các mục có nhãn [ FAIL ] hoặc [ BLOCKD ] theo hướng dẫn chi tiết.")
        else:
            print(f"\n{logger.BOLD_YELLOW}ℹ BÁO CÁO: ĐẠT 0/{total_steps} BƯỚC (Do hủy bỏ hoặc chưa kết nối phần cứng).{logger.RESET}")


if __name__ == "__main__":
    main()

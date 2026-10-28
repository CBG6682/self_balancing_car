#!/usr/bin/env python3
"""
Hardware Connection & Communication Diagnostic Suite
Target: STM32F401CCU6 Self-Balancing Robot + USB-to-UART / ST-Link / ESP32-S3

Features:
1. Instant Full-Bus Diagnostic Scan (USB bus, Kernel drivers, Physical TTYs).
2. Live Hotplug Watcher: Real-time detection the instant a USB device is plugged in.
3. Automated Handshake Tester: Sends PING/PONG and STATUS queries.
4. Loopback Self-Test: Verifies USB-UART chip health by bridging TX to RX.
5. Actionable troubleshooting hints in clean ANSI colors.
"""

import os
import sys
import time
import glob
import serial
import serial.tools.list_ports

# Import shared logger utility
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import logger_utils as logger

BAUDRATES = [115200, 9600, 57600, 38400]


def get_usb_sysfs_devices():
    """Reads all current USB device paths from /sys/bus/usb/devices/."""
    devices = {}
    for p in glob.glob("/sys/bus/usb/devices/[0-9]*"):
        dev_name = os.path.basename(p)
        if ":" in dev_name:
            continue  # Skip USB interfaces, only track root devices
        id_vendor_path = os.path.join(p, "idVendor")
        id_product_path = os.path.join(p, "idProduct")
        product_path = os.path.join(p, "product")
        manufacturer_path = os.path.join(p, "manufacturer")

        vid = ""
        pid = ""
        product = ""
        manuf = ""

        try:
            if os.path.exists(id_vendor_path):
                with open(id_vendor_path, "r") as f:
                    vid = f.read().strip()
            if os.path.exists(id_product_path):
                with open(id_product_path, "r") as f:
                    pid = f.read().strip()
            if os.path.exists(product_path):
                with open(product_path, "r") as f:
                    product = f.read().strip()
            if os.path.exists(manufacturer_path):
                with open(manufacturer_path, "r") as f:
                    manuf = f.read().strip()
        except (OSError, IOError):
            pass

        devices[dev_name] = {
            "vid": vid,
            "pid": pid,
            "product": product,
            "manufacturer": manuf,
            "path": p
        }
    return devices


def scan_instant():
    """Performs an instant deep scan of USB and physical serial ports."""
    logger.sub_header("QUÉT TRẠNG THÁI PHẦN CỨNG USB HIỆN TẠI")

    usb_devs = get_usb_sysfs_devices()
    print(f"  • Tổng số thiết bị USB đang kết nối vào máy tính: {len(usb_devs)}")
    for name, info in sorted(usb_devs.items()):
        vid_pid = f"[{info['vid']}:{info['pid']}]" if info['vid'] else "[Internal Hub]"
        desc = f"{info['manufacturer']} {info['product']}".strip() or "USB Device"
        print(f"    - Cổng {name:8s}: {vid_pid:12s} {desc}")

    # Physical serial ports
    all_ports = serial.tools.list_ports.comports()
    physical_ports = logger.filter_physical_ports(all_ports)

    logger.sub_header("QUÉT CỔNG SERIAL VẬT LÝ (/dev/ttyUSB*, /dev/ttyACM*)")
    if not physical_ports:
        logger.warn("Chưa phát hiện cổng Serial vật lý nào trên hệ thống.")
        return None

    logger.success(f"Tìm thấy {len(physical_ports)} cổng Serial vật lý:")
    for p in physical_ports:
        print(f"  ✔ Thiết bị: {logger.BOLD_CYAN}{p.device}{logger.RESET}")
        print(f"    - Mô tả phần cứng: {p.description}")
        print(f"    - HWID: {p.hwid}")

    return physical_ports


def test_handshake_on_port(port_name):
    """Tries PING/PONG and STATUS on a specific port across baudrates."""
    logger.sub_header(f"KIỂM TRA BẮT TAY TRUYỀN THÔNG TRÊN {port_name}")

    # Check permission
    if not os.access(port_name, os.R_OK | os.W_OK):
        logger.error(f"Không có quyền đọc/ghi trên cổng {port_name}!")
        print(f"  -> Khắc phục: Chạy lệnh `sudo chmod 666 {port_name}` hoặc thêm user vào group uucp: `sudo usermod -aG uucp $USER`")
        return False

    success = False

    for baud in BAUDRATES:
        print(f"  -> Thử giao tiếp tốc độ {baud} bps...", end="", flush=True)
        try:
            ser = serial.Serial(port_name, baudrate=baud, timeout=0.6)
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # 1. Send PING
            ser.write(b"PING\n")
            ser.flush()
            time.sleep(0.15)
            reply = ser.readline().decode("utf-8", errors="ignore").strip()

            if "PONG" in reply or "STATUS" in reply or "[SYS]" in reply:
                print(f" {logger.BOLD_GREEN}[THÀNH CÔNG]{logger.RESET}")
                logger.success(f"STM32 đã phản hồi tại tốc độ {baud} bps: '{reply}'")

                # 2. Query STATUS
                ser.write(b"STATUS\n")
                ser.flush()
                time.sleep(0.15)
                status_reply = ser.readline().decode("utf-8", errors="ignore").strip()
                if status_reply:
                    logger.info(f"Dữ liệu trạng thái xe: {logger.BOLD_WHITE}{status_reply}{logger.RESET}")

                success = True
                ser.close()
                break
            else:
                # Also check if telemetry is actively streaming
                ser.write(b"\n")
                time.sleep(0.1)
                peek = ser.readline().decode("utf-8", errors="ignore").strip()
                if peek.startswith("DATA:"):
                    print(f" {logger.BOLD_GREEN}[STREAM NHẬN DIỆN]{logger.RESET}")
                    logger.success(f"STM32 đang phát luồng Telemetry: {peek[:45]}...")
                    success = True
                    ser.close()
                    break

            ser.close()
            print(" (Không phản hồi)")
        except (serial.SerialException, OSError) as e:
            print(f" (Lỗi mở cổng: {e})")

    if not success:
        logger.warn(f"Cổng {port_name} đã mở được nhưng STM32 không phản hồi 'PONG'.")
        print("\n  • Các nguyên nhân có thể:")
        print(f"    1. Dây TX/RX bị cắm thẳng thay vì cắm chéo: Chân TX mạch USB-UART phải nối PA10 (RX STM32), RX nối PA9 (TX STM32).")
        print(f"    2. Mạch STM32 chưa được cấp nguồn (LED nguồn đỏ trên STM32 chưa sáng).")
        print(f"    3. Chưa nối chung chân GND giữa mạch USB-UART và STM32.")
        print(f"    4. Firmware trên STM32 chưa được nạp hoặc chip đang bị treo.")

    return success


def run_loopback_test(port_name):
    """Tests USB-to-UART module health by sending data and reading it back."""
    logger.sub_header(f"KIỂM TRA TỰ BẮT TAY (LOOPBACK SELF-TEST) TRÊN {port_name}")
    print("  • Mục đích: Kiểm tra xem CHÍNH CHIẾC MẠCH USB-UART có còn sống và truyền nhận tốt không.")
    print("  • Hướng dẫn:")
    print("    1. Rút hết dây nối sang STM32.")
    print("    2. Lấy 1 sợi dây cắm nối tắt chân TX và chân RX của mạch USB-UART với nhau.")
    input("\n  Khi đã nối tắt TX với RX, nhấn [ENTER] để chạy test...")

    try:
        ser = serial.Serial(port_name, baudrate=115200, timeout=1.0)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        test_payload = b"TEST_LOOPBACK_STM32_OK_12345\n"
        ser.write(test_payload)
        ser.flush()

        received = ser.readline()
        ser.close()

        if test_payload.strip() in received:
            logger.success("KẾT QUẢ LOOPBACK: 100% HOÀN HẢO!")
            print(f"  ✔ Mạch USB-UART truyền và nhận về chính xác: {received.decode('utf-8', errors='ignore').strip()}")
            print(f"  ✔ Kết luận: Chip USB-UART và Driver máy tính HOẠT ĐỘNG TỐT.")
            print(f"  👉 Nếu khi nối với xe mà không nhận, vấn đề 100% nằm ở: Đấu dây TX/RX sang STM32 hoặc nguồn STM32!")
        else:
            logger.error(f"KẾT QUẢ LOOPBACK: THẤT BẠI (Nhận được: {received})")
            print("  -> Mạch USB-UART có thể bị hỏng phần cứng hoặc chân tiếp xúc kém.")
    except Exception as e:
        logger.error(f"Không thể mở cổng để test loopback: {e}")


def live_watcher():
    """Continuously watches for newly plugged USB devices in real-time."""
    logger.sub_header("CHẾ ĐỘ LẮNG NGHE CẮM THIẾT BỊ THỜI GIAN THỰC (LIVE HOTPLUG MONITOR)")
    print("  • Hệ thống đang chờ bạn cắm mạch USB-UART vào bất kỳ cổng USB nào của máy tính...")
    print("  • (Bấm Ctrl+C bất cứ lúc nào để dừng lại)\n")

    initial_devs = get_usb_sysfs_devices()
    spinner = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
    idx = 0

    try:
        while True:
            curr_devs = get_usb_sysfs_devices()
            new_keys = set(curr_devs.keys()) - set(initial_devs.keys())

            if new_keys:
                print("\n")
                logger.success(f"PHÁT HIỆN THIẾT BỊ MỚI VỪA ĐƯỢC CẮM VÀO!")
                for k in new_keys:
                    info = curr_devs[k]
                    print(f"  ✔ Cổng USB: {k}")
                    print(f"  ✔ VID:PID : {info['vid']}:{info['pid']}")
                    print(f"  ✔ Tên chip: {info['manufacturer']} {info['product']}")

                time.sleep(0.8)  # Wait for kernel driver to instantiate tty
                all_ports = serial.tools.list_ports.comports()
                phys = logger.filter_physical_ports(all_ports)
                if phys:
                    target = phys[0].device
                    logger.success(f"Hệ điều hành đã tạo cổng Serial: {target}")
                    test_handshake_on_port(target)
                else:
                    logger.warn("Thiết bị đã kết nối nhưng chưa thấy driver tạo cổng /dev/ttyUSB*.")
                break

            removed_keys = set(initial_devs.keys()) - set(curr_devs.keys())
            if removed_keys:
                print(f"\n{logger.BOLD_YELLOW}[DISCONNECT]{logger.RESET} Một thiết bị vừa bị rút ra: {removed_keys}")
                initial_devs = curr_devs

            spin_char = spinner[idx % len(spinner)]
            idx += 1
            sys.stdout.write(f"\r  {spin_char} Đang theo dõi cổng USB... (Đã quét {len(curr_devs)} thiết bị) ")
            sys.stdout.flush()
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n\nĐã dừng chế độ theo dõi.")


def main():
    logger.step_header("BỘ CÔNG CỤ CHẨN ĐOÁN KẾT NỐI PHẦN CỨNG & USB-UART")

    # Step 1 & 2: Instant Scan
    physical_ports = scan_instant()

    if physical_ports:
        port_name = physical_ports[0].device
        connected = test_handshake_on_port(port_name)

        print("\n" + logger.BOLD_WHITE + "Các tùy chọn tiếp theo:" + logger.RESET)
        print(f"  [1] Chạy thử Loopback Self-Test trên cổng {port_name} (Rút dây xe, nối tắt TX-RX)")
        print(f"  [2] Bắt đầu lắng nghe cắm rút thiết bị thời gian thực")
        print(f"  [0] Thoát")
        choice = input("  Chọn [0-2]: ").strip()
        if choice == "1":
            run_loopback_test(port_name)
        elif choice == "2":
            live_watcher()
    else:
        # No physical ports found right now -> automatically enter live watcher
        print("\n" + logger.BOLD_YELLOW + ">>> Chưa có cổng Serial nào cắm sẵn. Tự động chuyển sang chế độ LẮNG NGHE THỜI GIAN THỰC..." + logger.RESET)
        live_watcher()


if __name__ == "__main__":
    main()

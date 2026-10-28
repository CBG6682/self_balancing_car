#!/usr/bin/env python3
"""
STM32 Hardware Verification & Anti-Echo Challenge Tool
Target: STM32F401CCU6 (USART1 PA9 TX, PA10 RX)
Distinguishes between genuine STM32 response vs fake ESP32 loopback echo.
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


def main():
    port_name = auto_detect_port()

    print("\n" + "=" * 70)
    print(" KIỂM TRA ĐƯỜNG TRUYỀN STM32 (CHỐNG NHẦM LẪN VỚI ESP32 DỘI NGƯỢC)")
    print(f" Cổng: {port_name} @ {BAUD} bps")
    print("=" * 70)
    print(" GIẢI THÍCH NGUYÊN LÝ CHỐNG NHẦM LẪN:")
    print("  • Nếu ESP32 bị dội ngược: Gửi chữ gì nó sẽ nhại lại đúng chữ đó.")
    print("  • Script này gửi chuỗi thử thách: 'PING_STM32_CHALLENGE'.")
    print("  • NẾU LÀ ESP32 TỰ DỘI: Nhận lại 'PING_STM32_CHALLENGE' -> Script báo LỖI!")
    print("  • NẾU LÀ STM32 THẬT TRẢ LỜI: Nhận được 'PONG_FROM_STM32_HARDWARE_OK'")
    print("    -> Xác nhận 100% chip STM32 đang hoạt động thật sự!")
    print("=" * 70 + "\n")

    try:
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = BAUD
        ser.timeout = 0.15
        ser.dtr = True
        ser.rts = True
        ser.open()
        time.sleep(1.0)
        ser.reset_input_buffer()
        logger.success(f"Cổng {port_name} đã mở thành công!")
    except Exception as e:
        logger.error(f"Không thể mở cổng {port_name}: {e}")
        print("  -> Vui lòng kiểm tra cáp cắm vào ESP32.")
        sys.exit(1)

    TOTAL_TESTS = 15
    stm32_real_count = 0
    esp32_echo_count = 0
    timeout_count = 0

    challenge_token = "PING_STM32_CHALLENGE"
    expected_stm32_ack = "PONG_FROM_STM32_HARDWARE_OK"

    logger.info(f"Đang gửi {TOTAL_TESTS} gói thử thách độc quyền tới STM32...")
    print("-" * 70)

    for i in range(1, TOTAL_TESTS + 1):
        ser.reset_input_buffer()
        ser.write((challenge_token + "\n").encode("utf-8"))
        ser.flush()

        time.sleep(0.08)
        received_line = ""

        # Đọc phản hồi
        start_t = time.time()
        while time.time() - start_t < 0.25:
            if ser.in_waiting:
                line = ser.readline().decode("latin1", errors="ignore").strip()
                if line:
                    received_line = line
                    break
            time.sleep(0.01)

        if expected_stm32_ack in received_line or "PONG" in received_line:
            stm32_real_count += 1
            print(f"  [Gói #{i:02d}]: {logger.BOLD_GREEN}STM32 PHẢN HỒI THẬT:{logger.RESET} '{received_line}'  {logger.BOLD_GREEN}[  PASS  ]{logger.RESET}")
        elif challenge_token in received_line:
            esp32_echo_count += 1
            print(f"  [Gói #{i:02d}]: {logger.BOLD_YELLOW}ESP32 TỰ DỘI NGƯỢC (ECHO):{logger.RESET} '{received_line}' {logger.BOLD_YELLOW}[ ESP ECHO ]{logger.RESET}")
        elif received_line:
            print(f"  [Gói #{i:02d}]: {logger.BOLD_RED}TÍN HIỆU LẠ/NHIỄU:{logger.RESET} '{received_line}' {logger.BOLD_RED}[ NOISE ]{logger.RESET}")
        else:
            timeout_count += 1
            print(f"  [Gói #{i:02d}]: {logger.DIM}(Không nhận được phản hồi){logger.RESET} {logger.BOLD_RED}[ TIMEOUT ]{logger.RESET}")

        time.sleep(0.05)

    print("-" * 70)
    ser.close()

    print("\n" + "╔" + "═" * 68 + "╗")
    print(f"║             BẢNG ĐỐI CHIẾU XÁC THỰC PHẦN CỨNG STM32                ║")
    print("╠" + "═" * 68 + "╣")
    print(f"║  • Tổng số gói thử thách:             {TOTAL_TESTS:<28} ║")
    print(f"║  • Số gói STM32 THẬT phản hồi:        {stm32_real_count:<28} ║")
    print(f"║  • Số gói do ESP32 tự dội ngược:     {esp32_echo_count:<28} ║")
    print(f"║  • Số gói mất tín hiệu (Timeout):     {timeout_count:<28} ║")
    print("╚" + "═" * 68 + "╝\n")

    if stm32_real_count >= 10:
        logger.success("XÁC NHẬN 100%: STM32 ĐANG GIAO TIẾP VÀ TRẢ LỜI THỰC SỰ!")
        print("  ✔ Cổng UART1 (PA9/PA10) của STM32 sống 100%!")
        print("  ✔ Không bị hiện tượng dội ngược của ESP32 đánh lừa.")
    elif esp32_echo_count > 0:
        logger.warn("CẢNH BÁO: CHÍNH XÁC NHƯ BẠN DỰ ĐOÁN, ESP32 ĐANG TỰ DỘI NGƯỢC!")
        print("  • ESP32 đang nhận lại đúng chữ nó phát ra (Chân 4 và 5 đang bị nối tắt hoặc chạm nhau).")
        print("  • STM32 chưa nhận được tín hiệu này.")
        print("\n  👉 KHUYÊN DÙNG: Hãy test STM32 ĐỘC LẬP bằng cách rút dây ESP32 ra, cắm thẳng PA9 vào PA10 và xem đèn LED PC13!")
    else:
        logger.error("Không có phản hồi nào từ cả ESP32 lẫn STM32.")


if __name__ == "__main__":
    main()

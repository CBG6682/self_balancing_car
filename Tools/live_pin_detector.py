#!/usr/bin/env python3
"""
Interactive Live Wire Detector:
Monitors both UART lines:
  Line 1: STM32 PA10 <---> ESP32 GPIO 6 (ESP32 TX -> STM32 RX)
  Line 2: STM32 PA9  <---> ESP32 GPIO 8 (STM32 TX -> ESP32 RX)
Prints real-time status as you touch/plug wires!
"""
import sys
import time
import serial
from pyocd.core.helpers import ConnectHelper

def main():
    print("==================================================================")
    print(" LIVE DUAL-UART WIRE DETECTOR")
    print(" Line 1: STM32 PA10 <---> ESP32 GPIO 6")
    print(" Line 2: STM32 PA9  <---> ESP32 GPIO 8")
    print("==================================================================")

    print("[1/2] Kết nối ESP32 trên /dev/ttyACM0...")
    try:
        esp = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
        esp.dtr = True; esp.rts = True
        time.sleep(0.2)
        esp.reset_input_buffer()
        print(" -> ESP32 đã kết nối!")
    except Exception as e:
        print(f" LỖI: {e}")
        return

    print("[2/2] Kết nối STM32 qua ST-Link...")
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m')
    session.open()
    target = session.board.target
    print(" -> STM32 đã kết nối!")

    GPIOA = 0x40020000
    MODER = 0x00
    PUPDR = 0x0C
    IDR   = 0x10
    BSRR  = 0x18

    orig_moder = target.read32(GPIOA + MODER)
    orig_pupdr = target.read32(GPIOA + PUPDR)

    def esp_status():
        esp.reset_input_buffer()
        esp.write(b'STATUS\n')
        time.sleep(0.01)
        line = esp.readline().decode(errors='replace').strip()
        parts = line.replace('PIN_STATUS:', '').strip().split()
        return {int(p.split('=')[0]): int(p.split('=')[1]) for p in parts if '=' in p}

    print("\n------------------------------------------------------------------")
    print(" Đang giám sát cả 2 đường truyền UART theo thời gian thực...")
    print(" Nhấn Ctrl+C để dừng.")
    print("------------------------------------------------------------------\n")

    try:
        count_both_ok = 0
        for cycle in range(120): # ~24 seconds
            # Test Line 1: ESP32 GPIO 6 -> STM32 PA10
            # Set PA10 to input with pull-down
            target.write32(GPIOA + MODER, orig_moder & ~(3 << 20))
            target.write32(GPIOA + PUPDR, (orig_pupdr & ~(3 << 20)) | (2 << 20))

            esp.write(b'PULL_HIGH 6\n')
            time.sleep(0.04)
            pa10_high = (target.read32(GPIOA + IDR) >> 10) & 1

            esp.write(b'PULL_LOW 6\n')
            time.sleep(0.04)
            pa10_low = (target.read32(GPIOA + IDR) >> 10) & 1

            line1_ok = (pa10_high == 1 and pa10_low == 0)

            # Test Line 2: STM32 PA9 -> ESP32 GPIO 8
            # Configure PA9 as output
            target.write32(GPIOA + MODER, (orig_moder & ~(3 << 18)) | (1 << 18))

            target.write32(GPIOA + BSRR, 1 << (9 + 16)) # PA9 LOW
            time.sleep(0.04)
            st_low = esp_status()
            gpio8_low = (st_low.get(8, 1) == 0)

            target.write32(GPIOA + BSRR, 1 << 9) # PA9 HIGH
            time.sleep(0.04)
            st_high = esp_status()
            gpio8_high = (st_high.get(8, 0) == 1)

            line2_ok = (gpio8_low and gpio8_high)

            # Format status
            s1 = "[OK - ĐÃ NỐI TỐT]" if line1_ok else "[LƠ LỬNG / ĐỨT DÂY]"
            s2 = "[OK - ĐÃ NỐI TỐT]" if line2_ok else "[LƠ LỬNG / ĐỨT DÂY]"

            if line1_ok and line2_ok:
                count_both_ok += 1
                print(f" [THÀNH CÔNG] Dây 1: {s1} | Dây 2: {s2} -> TOÀN BỘ 2 CHIỀU ĐÃ THÔNG TỐT!")
                if count_both_ok >= 3:
                    print("\n" + "=" * 65)
                    print(" HOÀN TẤT: CẢ 2 ĐƯỜNG TRUYỀN UART ĐÃ KẾT NỐI CHUẨN XÁC 100%!")
                    print("=" * 65 + "\n")
                    break
            else:
                count_both_ok = 0
                if cycle % 3 == 0:
                    print(f" Trạng thái: Dây 1 (PA10 <-> GPIO6): {s1:20} | Dây 2 (PA9 <-> GPIO8): {s2}")

    except KeyboardInterrupt:
        print("\nĐã dừng giám sát.")
    finally:
        target.write32(GPIOA + MODER, orig_moder)
        target.write32(GPIOA + PUPDR, orig_pupdr)
        esp.write(b'RELEASE 6\n')
        esp.write(b'RELEASE 8\n')
        session.close()
        esp.close()
        print("Đã hoàn tất dọn dẹp thiết bị.")

if __name__ == "__main__":
    main()

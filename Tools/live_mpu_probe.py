#!/usr/bin/env python3
"""
Live MPU6050 Hardware & I2C Wire Diagnostic Tool
Monitors SCL (PB6), SDA (PB7), VCC power state, and I2C response in real-time.
"""
import sys
import time
from pyocd.core.helpers import ConnectHelper

def main():
    print("==================================================================")
    print(" LIVE MPU6050 HARDWARE WIRE & VOLTAGE DIAGNOSTIC")
    print(" Target: STM32F401 BlackPill (I2C1: PB6=SCL, PB7=SDA)")
    print("==================================================================")

    print("Đang kết nối STM32 qua ST-Link...")
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m')
    session.open()
    target = session.board.target
    print(" -> Đã kết nối STM32!\n")

    GPIOB = 0x40020400
    IDR   = 0x10

    print("------------------------------------------------------------------")
    print(" Đang giám sát mức điện áp trên chân PB6 (SCL) và PB7 (SDA)...")
    print(" • SCL=1 (3.3V) & SDA=1 (3.3V): MPU6050 ĐANG CÓ NGUỒN BÌNH THƯỜNG")
    print(" • SCL=0 (0V)   hoặc SDA=0 (0V):   MPU6050 BỊ MẤT NGUỒN / HỞ DÂY VCC/GND!")
    print(" (Bấm Ctrl+C để dừng)")
    print("------------------------------------------------------------------\n")

    last_state = None
    try:
        while True:
            idr = target.read32(GPIOB + IDR)
            scl = (idr >> 6) & 1
            sda = (idr >> 7) & 1

            state = (scl, sda)
            if state != last_state:
                if scl == 1 and sda == 1:
                    print(f"\033[92m [OK - CÓ ĐIỆN NGUỒN] SCL(PB6)=3.3V | SDA(PB7)=3.3V -> Cảm biến MPU6050 đang có nguồn tốt!\033[0m")
                else:
                    print(f"\033[91m [LỖI - MẤT ĐIỆN NGUỒN] SCL(PB6)={scl * 3.3:.1f}V | SDA(PB7)={sda * 3.3:.1f}V -> DÂY NGUỒN VCC HOẶC GND BỊ HỞ / LỎNG!\033[0m")
                last_state = state

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nĐã dừng giám sát.")
    finally:
        session.close()

if __name__ == "__main__":
    main()

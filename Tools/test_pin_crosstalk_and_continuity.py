#!/usr/bin/env python3
"""
Hardware Pin Continuity, Short-Circuit, and Crosstalk Diagnostic Tool
For STM32F401CCU6 BlackPill <-> ESP32-S3 DevKitC-1
"""

import time
import sys
import serial
from pyocd.core.helpers import ConnectHelper

def main():
    print("==================================================================")
    print(" STM32F401 <-> ESP32-S3 PIN CONTINUITY & CROSSTALK DIAGNOSTIC")
    print("==================================================================")

    # 1. Connect to ESP32
    print("\n[1/5] Connecting to ESP32 on /dev/ttyACM0...")
    try:
        esp = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
        esp.dtr = True
        esp.rts = True
        time.sleep(0.5)
        # Flush initial boot junk
        esp.reset_input_buffer()
        print(" Connected to ESP32 CDC!")
    except Exception as e:
        print(f" ERROR opening ESP32: {e}")
        return

    # Helper function to send command and read reply from ESP32
    def esp_cmd(cmd, wait_time=0.1):
        esp.reset_input_buffer()
        esp.write((cmd + "\n").encode())
        esp.flush()
        time.sleep(wait_time)
        lines = []
        while esp.in_waiting:
            line = esp.readline().decode('utf-8', errors='replace').strip()
            if line:
                lines.append(line)
        return lines

    def esp_get_status():
        lines = esp_cmd("STATUS", 0.05)
        for line in lines:
            if line.startswith("PIN_STATUS:"):
                # Parse " 1=1 2=1 4=0 ..."
                parts = line.replace("PIN_STATUS:", "").strip().split()
                st = {}
                for p in parts:
                    if '=' in p:
                        k, v = p.split('=')
                        st[int(k)] = int(v)
                return st
        return {}

    # Helper function to find which ESP pins are currently LOW (0)
    def esp_pins_low():
        st = esp_get_status()
        return [pin for pin, val in st.items() if val == 0]

    # 2. Check ESP32 Solder Bridges & Internal Shorts
    print("\n[2/5] Scanning ESP32 for solder bridges and short circuits...")
    short_lines = esp_cmd("TEST_SHORTS", 0.5)
    shorts = [l for l in short_lines if l.startswith("SHORT:")]
    if shorts:
        print(" [WARNING] Solder bridges / short circuits detected on ESP32:")
        for s in shorts:
            print(f"   {s}")
    else:
        print(" [PASS] No short circuits found between any ESP32 GPIOs!")

    # 3. Connect to STM32 via pyocd
    print("\n[3/5] Connecting to STM32 via ST-Link...")
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m')
    session.open()
    board = session.board
    target = board.target
    print(" Connected to STM32 core!")

    # STM32 Register Addresses
    GPIOA_BASE = 0x40020000
    GPIOB_BASE = 0x40020400
    
    # Offsets
    MODER  = 0x00
    PUPDR  = 0x0C
    IDR    = 0x10
    ODR    = 0x14
    BSRR   = 0x18
    AFRH   = 0x24

    # Baseline status
    gpioa_idr = target.read32(GPIOA_BASE + IDR)
    gpiob_idr = target.read32(GPIOB_BASE + IDR)
    print(f" Initial STM32 GPIOA_IDR = 0x{gpioa_idr:08X}")
    print(f" Initial STM32 GPIOB_IDR = 0x{gpiob_idr:08X}")

    initial_low = esp_pins_low()
    print(f" Initial ESP32 pins reading LOW (0V): {initial_low if initial_low else 'None (all pulled HIGH)'}")

    # 4. TEST DIRECTION A: STM32 -> ESP32
    # Find which STM32 pin is connected to which ESP32 pin
    print("\n[4/5] Testing STM32 Pin Toggles -> Probing ESP32 GPIOs...")
    
    # Candidate STM32 pins on Header 2:
    # PA8, PA9, PA10, PA11, PA12, PA15, PB12, PB13, PB14, PB15, PB3, PB4, PB5, PB6, PB7, PB8, PB9
    stm32_test_pins = [
        ("GPIOA", GPIOA_BASE, 9,  "PA9 (USART1_TX default)"),
        ("GPIOA", GPIOA_BASE, 10, "PA10 (USART1_RX default)"),
        ("GPIOA", GPIOA_BASE, 8,  "PA8"),
        ("GPIOA", GPIOA_BASE, 11, "PA11"),
        ("GPIOA", GPIOA_BASE, 12, "PA12"),
        ("GPIOB", GPIOB_BASE, 9,  "PB9"),
        ("GPIOB", GPIOB_BASE, 8,  "PB8"),
        ("GPIOB", GPIOB_BASE, 10, "PB10"),
        ("GPIOB", GPIOB_BASE, 12, "PB12"),
        ("GPIOB", GPIOB_BASE, 13, "PB13"),
        ("GPIOB", GPIOB_BASE, 14, "PB14"),
        ("GPIOB", GPIOB_BASE, 15, "PB15"),
    ]

    found_stm_to_esp = []

    for port_name, port_base, pin_idx, pin_desc in stm32_test_pins:
        # Save original MODER, PUPDR, ODR
        orig_moder = target.read32(port_base + MODER)
        orig_odr   = target.read32(port_base + ODR)
        orig_pupdr = target.read32(port_base + PUPDR)

        # Configure pin as Output (MODER bits [2*pin+1 : 2*pin] = 0b01)
        new_moder = (orig_moder & ~(3 << (2 * pin_idx))) | (1 << (2 * pin_idx))
        target.write32(port_base + MODER, new_moder)

        # 1. Drive LOW
        target.write32(port_base + BSRR, 1 << (pin_idx + 16))
        time.sleep(0.02)
        lows_when_low = esp_pins_low()

        # 2. Drive HIGH
        target.write32(port_base + BSRR, 1 << pin_idx)
        time.sleep(0.02)
        lows_when_high = esp_pins_low()

        # Restore original configuration
        target.write32(port_base + MODER, orig_moder)
        target.write32(port_base + ODR, orig_odr)
        target.write32(port_base + PUPDR, orig_pupdr)

        # Pins that were LOW when STM32 drove LOW, but HIGH when STM32 drove HIGH
        detected = [p for p in lows_when_low if p not in lows_when_high]
        if detected:
            print(f" [CONNECTION FOUND!] STM32 {pin_desc} ----> ESP32 GPIO {detected}")
            found_stm_to_esp.append((pin_desc, detected))
        else:
            # print(f"   Tested STM32 {pin_desc}: no response on ESP32")
            pass

    if not found_stm_to_esp:
        print(" [INFO] No STM32 pin toggle was detected by any ESP32 pin.")
    else:
        print(f" Summary of detected STM32 -> ESP32 connections: {found_stm_to_esp}")

    # 5. TEST DIRECTION B: ESP32 -> STM32
    # Find which ESP32 pin pulls STM32 PA10 (or other pins) LOW
    print("\n[5/5] Testing ESP32 Pin Toggles -> Probing STM32 GPIO Inputs...")

    # Configure STM32 PA9, PA10, PA8, PA11, PB9, PB10 as inputs with PULLUP
    # GPIOA: PA8, PA9, PA10, PA11
    # GPIOB: PB8, PB9, PB10
    target.write32(GPIOA_BASE + PUPDR, target.read32(GPIOA_BASE + PUPDR) | (0x55 << 16)) # PA8..11 Pullup
    target.write32(GPIOB_BASE + PUPDR, target.read32(GPIOB_BASE + PUPDR) | (0x15 << 16)) # PB8..10 Pullup

    baseline_a = target.read32(GPIOA_BASE + IDR)
    baseline_b = target.read32(GPIOB_BASE + IDR)

    esp_candidate_pins = [4, 18, 5, 12, 11, 10, 9, 8, 7, 6, 3, 2, 1, 21, 43, 44, 38, 48]
    found_esp_to_stm = []

    for epin in esp_candidate_pins:
        # Drive ESP pin LOW
        esp_cmd(f"PULL_LOW {epin}", 0.02)
        cur_a = target.read32(GPIOA_BASE + IDR)
        cur_b = target.read32(GPIOB_BASE + IDR)
        
        # Release ESP pin back to INPUT_PULLUP
        esp_cmd(f"RELEASE {epin}", 0.02)

        # Check which bits went from 1 to 0
        diff_a = (baseline_a ^ cur_a) & baseline_a
        diff_b = (baseline_b ^ cur_b) & baseline_b

        for bit in range(16):
            if diff_a & (1 << bit):
                print(f" [CONNECTION FOUND!] ESP32 GPIO {epin} ----> STM32 PA{bit} (Bit pulled LOW!)")
                found_esp_to_stm.append((epin, f"PA{bit}"))
            if diff_b & (1 << bit):
                print(f" [CONNECTION FOUND!] ESP32 GPIO {epin} ----> STM32 PB{bit} (Bit pulled LOW!)")
                found_esp_to_stm.append((epin, f"PB{bit}"))

    if not found_esp_to_stm:
        print(" [INFO] No ESP32 pin was able to pull any STM32 pin LOW.")
    else:
        print(f" Summary of detected ESP32 -> STM32 connections: {found_esp_to_stm}")

    session.close()
    esp.close()

    print("\n==================================================================")
    print(" DIAGNOSTIC COMPLETE")
    print("==================================================================")

if __name__ == "__main__":
    main()

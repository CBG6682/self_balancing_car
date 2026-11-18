#!/usr/bin/env python3
"""
==============================================================================
Interactive ESP32 Live Tuning & Balance Monitor with CSV Export
Target: STM32F401CCU6 via ESP32-S3 BLE Bridge (/dev/ttyACM0)
==============================================================================
"""

import sys
import os
import time
import csv
import math
import argparse
import serial
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAVE_MATPLOTLIB = True
except ImportError:
    HAVE_MATPLOTLIB = False

PORT = "/dev/ttyACM0"
BAUD = 115200

# Physical constants for GA25 + 65mm wheels
ENCODER_CPR = 1320
WHEEL_RADIUS_METERS = 0.0325
METERS_PER_PULSE = (2.0 * math.pi * WHEEL_RADIUS_METERS) / ENCODER_CPR

def send_cmd(ser, cmd_str):
    ser.reset_input_buffer()
    ser.write((cmd_str.strip() + "\n").encode())
    time.sleep(0.05)
    resp = ser.readline().decode("utf-8", errors="ignore").strip()
    return resp

def main():
    parser = argparse.ArgumentParser(description="Interactive ESP32 Live Tuning Monitor with CSV Export")
    parser.add_argument("--port", default=PORT, help="Serial port (/dev/ttyACM0)")
    parser.add_argument("--kp", type=float, default=77.0, help="Angle Kp")
    parser.add_argument("--ki", type=float, default=0.006, help="Angle Ki")
    parser.add_argument("--kd", type=float, default=0.800, help="Angle Kd")
    parser.add_argument("--skp", type=float, default=0.45, help="Speed Kp")
    parser.add_argument("--ski", type=float, default=0.010, help="Speed Ki")
    parser.add_argument("--offset", type=float, default=0.32, help="Balance Offset (deg)")
    parser.add_argument("--duration", type=float, default=15.0, help="Monitor duration (s)")
    parser.add_argument("--csv", default="Tools/test_15s_session.csv", help="CSV export filename")
    parser.add_argument("--plot", default="test_15s_session_plot.png", help="Plot export filename")
    args = parser.parse_args()

    print("\n" + "═" * 70)
    print("      CHƯƠNG TRÌNH TEST CÂN BẰNG XE TRỰC TIẾP QUA ESP32 (HOT-TUNE)")
    print("═" * 70)
    print(f"  Cổng giao tiếp : {args.port} @ 115200")
    print(f"  Thời gian test : {args.duration:.1f} giây")
    print(f"  Thông số nạp RAM: Kp={args.kp:.2f}, Ki={args.ki:.3f}, Kd={args.kd:.3f}, sKp={args.skp:.2f}, sKi={args.ski:.3f}, Offset={args.offset:+.2f}°")
    print(f"  File CSV xuất  : {args.csv}")
    print("═" * 70 + "\n")

    try:
        ser = serial.Serial(args.port, BAUD, timeout=0.2)
    except Exception as e:
        print(f"[ERR] Không thể kết nối tới {args.port}: {e}")
        sys.exit(1)

    time.sleep(0.1)
    ser.write(b"STOP_STREAM\n")
    time.sleep(0.1)

    # Hot-load parameters into RAM
    r_pid = send_cmd(ser, f"SET_PID:{args.kp:.2f},{args.ki:.3f},{args.kd:.3f},{args.skp:.2f},{args.ski:.3f}")
    r_off = send_cmd(ser, f"SET_OFFSET:{args.offset:.2f}")
    r_mode = send_cmd(ser, "AUTO")
    r_curr = send_cmd(ser, "GET_PID")

    print(f"  [RAM NẠP] {r_pid}")
    print(f"  [RAM NẠP] {r_off}")
    print(f"  [CHẾ ĐỘ ] {r_mode}")
    print(f"  [HIỆN TẠI] {r_curr}")

    print("\n" + "─" * 70)
    print("  HÃY DỰNG XE THẲNG ĐỨNG LÊN 2 BÁNH (DÂY CÁP THẢ LỎNG TRÊN BÀN/SÀN)")
    print("  Khi góc nghiêng vào vùng +/-10°, xe sẽ tự động bật cân bằng.")
    print("─" * 70 + "\n")

    ser.write(b"START_STREAM\n")
    time.sleep(0.05)

    all_records = []
    attempts = []
    curr_attempt = []
    start_t = time.time()
    last_ui_t = 0
    in_balancing = False

    try:
        while (time.time() - start_t) < args.duration:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line or not line.startswith("DATA:"):
                continue

            parts = line[5:].split(",")
            if len(parts) < 6:
                continue

            t_now = time.time() - start_t
            p = float(parts[0])
            g = float(parts[1])
            pwm = int(parts[2])
            el = int(parts[3])
            er = int(parts[4])
            st = int(parts[5])

            sample = {
                "time": f"{t_now:.3f}",
                "pitch_angle": p,
                "gyro_rate": g,
                "pwm_command": pwm,
                "enc_left": el,
                "enc_right": er,
                "fsm_state": st
            }
            all_records.append(sample)

            if st == 1:
                if not in_balancing:
                    in_balancing = True
                    curr_attempt = []
                    print(f"\n  >>> [BẮT ĐẦU CÂN BẰNG] Lúc t={t_now:.2f}s | Pitch khởi điểm={p:+.2f}°")
                curr_attempt.append(sample)
            else:
                if in_balancing:
                    in_balancing = False
                    if len(curr_attempt) > 10:
                        attempts.append(list(curr_attempt))
                        dur = len(curr_attempt) * 0.01
                        print(f"\n  <<< [KẾT THÚC CÂN BẰNG] Duy trì được: {dur:.2f} giây | Pitch lúc ngã={p:+.2f}°")
                    curr_attempt = []

            # UI refresh every 100ms
            if (t_now - last_ui_t) >= 0.10:
                last_ui_t = t_now
                bar_w = 15
                clamped_p = max(-15.0, min(15.0, p))
                pos = int((clamped_p + 15.0) / 30.0 * (bar_w * 2))
                gauge = list("-" * (bar_w * 2 + 1))
                gauge[bar_w] = "|"
                if 0 <= pos < len(gauge):
                    gauge[pos] = "O"
                gauge_str = "".join(gauge)

                st_str = "BALANCING" if st == 1 else "IDLE/OFF"
                rem = args.duration - t_now
                sys.stdout.write(f"\r[{rem:4.1f}s còn lại] [{gauge_str}] Pitch: {p:+6.2f}° | Gyro: {g:+6.1f}°/s | PWM: {pwm:+5d} | {st_str:9s} ")
                sys.stdout.flush()

    finally:
        ser.write(b"STOP_STREAM\n")
        time.sleep(0.05)
        ser.close()

    if in_balancing and len(curr_attempt) > 10:
        attempts.append(list(curr_attempt))

    print("\n\n" + "═" * 70)
    print(f"  TỔNG KẾT PHIÊN THỬ NGHIỆM: Ghi nhận {len(attempts)} lần đứng cân bằng | Tổng số mẫu: {len(all_records)}")
    print("═" * 70)

    # Write CSV
    if all_records:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
        with open(args.csv, "w", newline="") as f:
            fieldnames = ["time", "pitch_angle", "gyro_rate", "pwm_command", "enc_left", "enc_right", "fsm_state"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_records)
        print(f"\n[XUẤT DỮ LIỆU] Đã lưu toàn bộ {len(all_records)} mẫu vào file: {args.csv}")

    if not attempts:
        print("  [CHƯA CÓ LẦN CÂN BẰNG NÀO] Xe chưa vào góc an toàn +/-10° trong thời gian test.")
        print("  Hãy thử lại và dựng xe thẳng đứng hơn.")
        return

    for idx, att in enumerate(attempts, 1):
        dur = len(att) * 0.01
        ps = [s["pitch_angle"] for s in att]
        gs = [s["gyro_rate"] for s in att]
        pwms = [s["pwm_command"] for s in att]
        mean_p = sum(ps) / len(ps)
        rms_p = math.sqrt(sum(x*x for x in ps) / len(ps))
        mean_pwm = sum(pwms) / len(pwms)
        fall_pitch = ps[-1]

        # Chatter flips
        flips = sum(1 for i in range(1, len(pwms)) if (pwms[i]>10 and pwms[i-1]<-10) or (pwms[i]<-10 and pwms[i-1]>10))
        chatter_hz = flips / dur if dur > 0 else 0

        d_el = att[-1]["enc_left"] - att[0]["enc_left"]
        d_er = att[-1]["enc_right"] - att[0]["enc_right"]
        drift_cm = ((d_el + d_er) * 0.5) * METERS_PER_PULSE * 100.0

        print(f"\n  ── Lần cân bằng #{idx} ({dur:.2f} giây) ──")
        print(f"     • Góc trung bình (Mean Pitch) : {mean_p:+6.2f}° (Độ rung RMS: {rms_p:.2f}°)")
        print(f"     • Lực kéo trung bình (Mean PWM): {mean_pwm:+6.1f}")
        print(f"     • Tần số đảo chiều (Chatter)  : {chatter_hz:6.1f} Hz")
        print(f"     • Dịch chuyển vị trí (Drift)  : {drift_cm:+6.1f} cm (Trái: {d_el:+d}, Phải: {d_er:+d})")
        print(f"     • Chiều ngã cuối cùng         : {'TIẾN (Forward)' if fall_pitch > 0 else 'LÙI (Backward)'} @ {fall_pitch:+.2f}°")

        # Diagnosis
        print("     • Đánh giá & Hướng điều chỉnh:")
        if dur >= 5.0:
            print("       ★ XUẤT SẮC! Xe đã cân bằng ổn định trên 5 giây!")
        elif dur >= 2.0:
            print("       ✓ Khá tốt! Xe đã giữ được thăng bằng cơ bản.")
        else:
            print("       ! Thời gian còn ngắn, cần tinh chỉnh độ cứng hoặc trọng tâm.")

    # Plot
    if HAVE_MATPLOTLIB and len(all_records) > 50:
        times = [float(r["time"]) for r in all_records]
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axs[0].plot(times, [r["pitch_angle"] for r in all_records], label="Pitch Angle (°)", color="tab:blue")
        axs[0].axhline(0, color="gray", linestyle="--", alpha=0.5)
        axs[0].axhline(args.offset, color="red", linestyle=":", label=f"Offset ({args.offset:+.2f}°)")
        axs[0].set_ylabel("Pitch (°)")
        axs[0].set_title(f"Góc Nghiêng Thân Xe - Test 15s (Kp={args.kp}, Kd={args.kd}, Offset={args.offset}°)")
        axs[0].legend(loc="upper right")
        axs[0].grid(True, alpha=0.3)

        axs[1].plot(times, [r["gyro_rate"] for r in all_records], label="Gyro (°/s)", color="tab:orange")
        axs[1].axhline(0, color="gray", linestyle="--", alpha=0.5)
        axs[1].set_ylabel("Gyro (°/s)")
        axs[1].legend(loc="upper right")
        axs[1].grid(True, alpha=0.3)

        axs[2].plot(times, [r["pwm_command"] for r in all_records], label="PWM Command", color="tab:green")
        axs[2].axhline(0, color="gray", linestyle="--", alpha=0.5)
        axs[2].set_ylabel("PWM Duty")
        axs[2].set_xlabel("Thời gian (giây)")
        axs[2].legend(loc="upper right")
        axs[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(args.plot, dpi=120)
        plt.close()
        print(f"[ĐỒ THỊ] Đã lưu biểu đồ phân tích 15s vào: {args.plot}")

    print("\n" + "═" * 70 + "\n")

if __name__ == "__main__":
    main()

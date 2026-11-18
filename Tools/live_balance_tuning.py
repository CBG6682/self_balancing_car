#!/usr/bin/env python3
"""
==============================================================================
Live Balance Telemetry Collector & Parameter Optimizer
Target: STM32F401CCU6 BlackPill + ESP32-S3 BLE Bridge
==============================================================================
"""

import sys
import os
import time
import csv
import math
import argparse
import serial
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PORT = "/dev/ttyACM0"
BAUD = 115200
DEFAULT_DURATION = 8.0
CSV_OUT = "Tools/live_untethered_run.csv"
PLOT_OUT = "untethered_balance_analysis.png"

# Physical constants for GA25 + 65mm wheels
ENCODER_CPR = 1320
WHEEL_RADIUS_METERS = 0.0325
METERS_PER_PULSE = (2.0 * math.pi * WHEEL_RADIUS_METERS) / ENCODER_CPR

def main():
    parser = argparse.ArgumentParser(description="Live Telemetry & Balance Optimizer")
    parser.add_argument("--port", default=PORT, help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=BAUD, help="Baud rate (default: 115200)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION, help="Recording duration in seconds")
    parser.add_argument("--kp", type=float, default=58.0, help="Initial Angle Kp")
    parser.add_argument("--ki", type=float, default=0.010, help="Initial Angle Ki")
    parser.add_argument("--kd", type=float, default=0.720, help="Initial Angle Kd")
    parser.add_argument("--skp", type=float, default=2.40, help="Initial Speed Kp")
    parser.add_argument("--ski", type=float, default=0.080, help="Initial Speed Ki")
    parser.add_argument("--offset", type=float, default=0.00, help="Initial Balance Offset (deg)")
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("      THU THẬP DỮ LIỆU CÂN BẰNG THỰC TẾ & TỐI ƯU HÓA THÔNG SỐ")
    print("=" * 70)
    print(f"  Cổng kết nối : {args.port} @ {args.baud} baud")
    print(f"  Thời gian ghi: {args.duration} giây")
    print(f"  Thông số nạp : Kp={args.kp:.1f}, Ki={args.ki:.3f}, Kd={args.kd:.3f}, sKp={args.skp:.2f}, sKi={args.ski:.3f}, Offset={args.offset:+.2f}°")
    print("=" * 70 + "\n")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.3)
    except Exception as e:
        print(f"[ERR] Không thể kết nối tới {args.port}: {e}")
        sys.exit(1)

    time.sleep(0.1)
    ser.write(b"STOP_STREAM\n")
    time.sleep(0.1)
    ser.reset_input_buffer()

    # 1. Apply baseline gains to RAM
    cmd_pid = f"SET_PID:{args.kp:.2f},{args.ki:.3f},{args.kd:.3f},{args.skp:.2f},{args.ski:.3f}\n".encode()
    ser.write(cmd_pid)
    time.sleep(0.05)
    ser.write(f"SET_OFFSET:{args.offset:.2f}\n".encode())
    time.sleep(0.05)
    ser.write(b"AUTO\n")
    time.sleep(0.05)
    ser.write(b"GET_PID\n")
    time.sleep(0.05)
    ack_lines = ser.read_all().decode("utf-8", errors="ignore").strip().splitlines()
    for l in ack_lines:
        print(f"  [STM32 ACK] {l}")

    print("\n" + "-" * 70)
    print("  LƯU Ý QUAN TRỌNG ĐỂ CÓ SỐ LIỆU CHÍNH XÁC:")
    print("  1. Hãy để dây cáp USB CHÙNG XUỐNG BÀN/SÀN (không được kéo căng xe)")
    print("  2. Đặt xe đứng thẳng lên 2 bánh xe")
    print("  3. Thả tay nhẹ nhàng khi xe bắt đầu kích hoạt chế độ cân bằng")
    print("-" * 70 + "\n")

    # Start stream
    ser.write(b"START_STREAM\n")
    time.sleep(0.05)

    records = []
    start_t = time.time()
    last_ui_t = 0
    bal_count = 0

    try:
        while (time.time() - start_t) < args.duration:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("DATA:"):
                parts = line[5:].split(",")
                if len(parts) >= 6:
                    t_curr = time.time() - start_t
                    p = float(parts[0])
                    g = float(parts[1])
                    pwm = int(parts[2])
                    el = int(parts[3])
                    er = int(parts[4])
                    st = int(parts[5])

                    if st == 1:
                        bal_count += 1

                    records.append({
                        "time": f"{t_curr:.3f}",
                        "pitch_angle": p,
                        "gyro_rate": g,
                        "pwm_command": pwm,
                        "enc_left": el,
                        "enc_right": er,
                        "fsm_state": st
                    })

                    # Live UI gauge bar every 100ms
                    if (t_curr - last_ui_t) >= 0.10:
                        last_ui_t = t_curr
                        # ASCII gauge for pitch (-15° to +15°)
                        bar_w = 15
                        clamped_p = max(-15.0, min(15.0, p))
                        pos = int((clamped_p + 15.0) / 30.0 * (bar_w * 2))
                        gauge = list("-" * (bar_w * 2 + 1))
                        gauge[bar_w] = "|" # Center mark
                        if 0 <= pos < len(gauge):
                            gauge[pos] = "O"
                        gauge_str = "".join(gauge)

                        st_label = "BALANCING" if st == 1 else "IDLE/OFF"
                        pct = int((t_curr / args.duration) * 100)
                        sys.stdout.write(f"\r[{pct:3d}%] [{gauge_str}] Pitch: {p:+6.2f}° | Gyro: {g:+6.1f}°/s | PWM: {pwm:+5d} | {st_label}   ")
                        sys.stdout.flush()

    finally:
        ser.write(b"STOP_STREAM\n")
        time.sleep(0.1)
        ser.close()

    print("\n\n" + "=" * 70)
    print(f"[HOÀN TẤT THU THẬP] Tổng số mẫu: {len(records)} | Mẫu cân bằng (State=1): {bal_count}")
    print("=" * 70 + "\n")

    if not records:
        print("[ERR] Không nhận được dữ liệu nào từ STM32!")
        return

    # Save CSV
    with open(CSV_OUT, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "pitch_angle", "gyro_rate", "pwm_command", "enc_left", "enc_right", "fsm_state"])
        writer.writeheader()
        writer.writerows(records)
    print(f"[LƯU TRỮ] Đã lưu dữ liệu chi tiết vào file: {CSV_OUT}")

    # Analyze data
    bal_records = [r for r in records if r["fsm_state"] == 1]
    if len(bal_records) < 50:
        print("\n[CẢNH BÁO] Số mẫu cân bằng còn ít (< 50 mẫu). Xe có thể chưa được dựng đứng hoặc bị ngã sớm.")
        print("Dưới đây là thống kê trên toàn bộ dữ liệu ghi nhận được:")
        analyze_target = records
    else:
        print(f"\n[PHÂN TÍCH CHUYÊN SÂU] Phân tích trên {len(bal_records)} mẫu cân bằng liên tục:")
        analyze_target = bal_records

    times = [float(r["time"]) for r in analyze_target]
    pitches = [r["pitch_angle"] for r in analyze_target]
    gyros = [r["gyro_rate"] for r in analyze_target]
    pwms = [r["pwm_command"] for r in analyze_target]
    enc_l = [r["enc_left"] for r in analyze_target]
    enc_r = [r["enc_right"] for r in analyze_target]

    mean_pitch = sum(pitches) / len(pitches)
    rms_pitch = math.sqrt(sum(p * p for p in pitches) / len(pitches))
    max_pitch = max(abs(p) for p in pitches)

    mean_gyro = sum(gyros) / len(gyros)
    rms_gyro = math.sqrt(sum(g * g for g in gyros) / len(gyros))

    mean_pwm = sum(pwms) / len(pwms)
    rms_pwm = math.sqrt(sum(u * u for u in pwms) / len(pwms))

    # Calculate PWM sign flips (mechanical chattering frequency)
    sign_flips = 0
    for i in range(1, len(pwms)):
        if (pwms[i] > 10 and pwms[i-1] < -10) or (pwms[i] < -10 and pwms[i-1] > 10):
            sign_flips += 1
    duration_active = (times[-1] - times[0]) if len(times) > 1 else 1.0
    chatter_freq_hz = sign_flips / duration_active if duration_active > 0 else 0

    # Calculate position drift
    delta_l = enc_l[-1] - enc_l[0]
    delta_r = enc_r[-1] - enc_r[0]
    drift_m = ((delta_l + delta_r) * 0.5) * METERS_PER_PULSE
    drift_speed_mps = drift_m / duration_active if duration_active > 0 else 0

    # Optimal balance offset calculation:
    # At true neutral COM, the steady-state required motor PWM is 0.
    # If mean_pwm != 0, it means the car is leaning against gravity.
    # Delta_offset = mean_pwm / Kp
    offset_correction_deg = mean_pwm / args.kp if args.kp > 0 else 0
    recommended_offset = args.offset - offset_correction_deg

    print("\n" + "─" * 60)
    print("  KẾT QUẢ ĐO ĐẠC ĐẶC TÍNH VẬT LÝ:")
    print("─" * 60)
    print(f"  • Góc nghiêng trung bình (Mean Pitch) : {mean_pitch:+6.2f}°")
    print(f"  • Độ lệch chuẩn dao động (RMS Pitch)  : {rms_pitch:6.2f}°")
    print(f"  • Tốc độ góc con quay (RMS Gyro)      : {rms_gyro:6.2f}°/s")
    print(f"  • Lực kéo động cơ trung bình (Mean PWM): {mean_pwm:+6.1f} (Tối đa: 3359)")
    print(f"  • Tần số đảo chiều giật (Chatter Freq) : {chatter_freq_hz:6.1f} Hz")
    print(f"  • Quãng đường trôi dạt (Total Drift)  : {drift_m:+6.3f} m ({drift_m*100:+.1f} cm)")
    print(f"  • Vận tốc trôi dạt (Drift Velocity)   : {drift_speed_mps*100:+6.2f} cm/s")
    print("─" * 60)

    # Recommendations
    rec_kp = args.kp
    rec_kd = args.kd
    rec_ki = args.ki
    rec_skp = args.skp
    rec_ski = args.ski

    # If chatter frequency > 8 Hz, Kd is too high or gyro noise needs lower gain
    if chatter_freq_hz > 8.0:
        rec_kd = max(0.55, args.kd * 0.85)
        print(f"  [KHẮC PHỤC GIẬT] Tần số giật {chatter_freq_hz:.1f}Hz > 8Hz: Đề xuất giảm Kd từ {args.kd:.3f} -> {rec_kd:.3f}")

    # If drift velocity > 3 cm/s, strengthen speed loop
    if abs(drift_speed_mps) > 0.03:
        rec_skp = min(3.5, args.skp * 1.25)
        rec_ski = min(0.15, args.ski * 1.30)
        print(f"  [KHẮC PHỤC TRÔI] Vận tốc trôi {abs(drift_speed_mps)*100:.1f}cm/s > 3cm/s: Đề xuất tăng sKp={rec_skp:.2f}, sKi={rec_ski:.3f}")

    # If mean PWM has strong DC bias, adjust balance offset
    if abs(offset_correction_deg) >= 0.10:
        print(f"  [ĐIỀU CHỈNH TRỌNG TÂM] Phát hiện lực kéo lệch {mean_pwm:+.1f} PWM: Đề xuất chỉnh Offset từ {args.offset:+.2f}° -> {recommended_offset:+.2f}°")
    else:
        recommended_offset = args.offset

    print("─" * 60)
    print(f"  BỘ THÔNG SỐ TỐI ƯU KHUYẾN NGHỊ:")
    print(f"  Angle Kp = {rec_kp:.2f} | Ki = {rec_ki:.3f} | Kd = {rec_kd:.3f}")
    print(f"  Speed Kp = {rec_skp:.2f} | Ki = {rec_ski:.3f}")
    print(f"  Balance Offset = {recommended_offset:+.2f}°")
    print("─" * 60 + "\n")

    # Generate Analysis Plot
    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    all_times = [float(r["time"]) for r in records]

    # Subplot 1: Pitch Angle
    axs[0].plot(all_times, [r["pitch_angle"] for r in records], label="Pitch Angle (°)", color="tab:blue", lw=1.5)
    axs[0].axhline(0, color="gray", linestyle="--", alpha=0.6)
    axs[0].axhline(recommended_offset, color="tab:red", linestyle=":", label=f"True COM ({recommended_offset:+.2f}°)")
    axs[0].set_ylabel("Pitch Angle (°)")
    axs[0].set_title("1. Góc Nghiêng Thân Xe (Pitch Angle Stability)")
    axs[0].legend(loc="upper right")
    axs[0].grid(True, alpha=0.3)

    # Subplot 2: Gyro Rate
    axs[1].plot(all_times, [r["gyro_rate"] for r in records], label="Gyro Rate (°/s)", color="tab:orange", lw=1.2)
    axs[1].axhline(0, color="gray", linestyle="--", alpha=0.6)
    axs[1].set_ylabel("Gyro Rate (°/s)")
    axs[1].set_title("2. Tốc Độ Góc & Rung Cơ Học Hộp Số (GA25 Backlash Damping)")
    axs[1].legend(loc="upper right")
    axs[1].grid(True, alpha=0.3)

    # Subplot 3: Motor PWM Command
    axs[2].plot(all_times, [r["pwm_command"] for r in records], label="Motor PWM", color="tab:green", lw=1.2)
    axs[2].axhline(0, color="gray", linestyle="--", alpha=0.6)
    axs[2].set_ylabel("PWM Duty")
    axs[2].set_title(f"3. Lực Kéo Động Cơ (Chatter Rate: {chatter_freq_hz:.1f} Hz)")
    axs[2].legend(loc="upper right")
    axs[2].grid(True, alpha=0.3)

    # Subplot 4: Wheel Encoders (Position Drift)
    axs[3].plot(all_times, [r["enc_left"] for r in records], label="Encoder Left", color="tab:purple", lw=1.2)
    axs[3].plot(all_times, [r["enc_right"] for r in records], label="Encoder Right", color="tab:brown", linestyle="--", lw=1.2)
    axs[3].set_ylabel("Encoder Pulses")
    axs[3].set_xlabel("Thời gian (giây)")
    axs[3].set_title(f"4. Dịch Chuyển Bánh Xe & Trôi Dạt (Total Drift: {drift_m*100:+.1f} cm)")
    axs[3].legend(loc="upper right")
    axs[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(PLOT_OUT, dpi=120)
    plt.close()
    print(f"[ĐỒ THỊ] Đã vẽ và lưu biểu đồ phân tích vào: {PLOT_OUT}")

if __name__ == "__main__":
    main()

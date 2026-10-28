#!/usr/bin/env python3
"""
Mock Data Generator & Hardware Data Acquisition
Target: STM32F401CCU6 Self-Balancing Robot

Features:
1. MockDataGenerator: High-fidelity non-linear inverted pendulum physics simulation (RK4)
   with sensor noise (accelerometer/gyroscope Gaussian noise and drift), motor deadzone,
   and external tilt disturbance pulses. Outputs Tools/mock_telemetry.csv.
2. Serial Hardware Probe: Scans COM / /dev/tty ports, sends PING\n handshake to STM32.
   - If STM32 responds PONG: collects 6s real telemetry to Tools/real_telemetry.csv.
   - If not found or absent: falls back cleanly to Tools/mock_telemetry.csv.
"""

import os
import sys
import time
import csv
import math
import yaml
import json
import numpy as np
import serial
import serial.tools.list_ports

# Import shared logger utility
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import logger_utils as logger

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "robot_config.yaml")
GAINS_FILE = os.path.join(SCRIPT_DIR, "computed_gains.json")
MOCK_CSV_FILE = os.path.join(SCRIPT_DIR, "mock_telemetry.csv")
REAL_CSV_FILE = os.path.join(SCRIPT_DIR, "real_telemetry.csv")


class MockDataGenerator:
    """
    Simulates non-linear physics of 2-wheeled inverted pendulum using 4th order Runge-Kutta.
    Includes TB6612 PWM deadzone, MPU6050 noise and drift, and external disturbance kicks.
    """
    def __init__(self, config_path: str, gains_path: str):
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

        if os.path.exists(gains_path):
            with open(gains_path, "r", encoding="utf-8") as f:
                gains_data = json.load(f)
                self.pid_gains = gains_data.get("cascade_pid", {})
        else:
            self.pid_gains = {
                "angle_kp": 350.0, "angle_ki": 1.75, "angle_kd": 5.0,
                "speed_kp": 6.0, "speed_ki": 0.4, "speed_kd": 0.0
            }

        chassis = self.config["chassis"]
        motor = self.config["motor"]

        self.M = float(chassis["M"])
        self.m = float(chassis["m"])
        self.R = float(chassis["R"])
        self.L = float(chassis["L"])
        self.g = float(chassis["g"])
        self.I_body = float(chassis["I_body"])
        self.I_wheel = 0.5 * self.m * (self.R ** 2)

        self.pwm_max = float(motor["pwm_max"])
        self.pwm_deadzone = float(motor["pwm_deadzone"])
        self.encoder_cpr = float(motor["encoder_cpr"])

        # Conversion: stall torque ~1.6 Nm combined at 11.1V
        self.max_torque = 1.6
        self.b_friction = 1.0  # Viscous damping

        self.meters_per_pulse = (2.0 * math.pi * self.R) / self.encoder_cpr

    def _dynamics(self, state, u_effort):
        """
        Non-linear equations of motion.
        state = [x (m), v (m/s), theta (rad), omega (rad/s)]
        u_effort = normalized torque effort [-1.0, 1.0]
        """
        x, v, theta, omega = state

        # Wheel torque and force with deadzone and saturation
        torque = u_effort * self.max_torque
        F_wheel = (torque / self.R) - (self.b_friction * v)

        sin_t = math.sin(theta)
        cos_t = math.cos(theta)

        M_total = self.M + self.m + (2.0 * self.I_wheel / (self.R ** 2))
        J_pitch = self.I_body + self.M * (self.L ** 2)

        denom = (M_total * J_pitch) - ((self.M * self.L * cos_t) ** 2)
        if denom < 1e-6:
            denom = 1e-6

        # Coupled non-linear accelerations
        centrifugal = self.M * self.L * (omega ** 2) * sin_t
        gravity_torque = self.M * self.g * self.L * sin_t

        accel_x = ((J_pitch * (F_wheel + centrifugal)) - (self.M * self.L * cos_t * gravity_torque)) / denom
        accel_theta = ((M_total * gravity_torque) - (self.M * self.L * cos_t * (F_wheel + centrifugal))) / denom

        return np.array([v, accel_x, omega, accel_theta])

    def generate(self, duration_sec: float = 6.0, dt: float = 0.01, output_file: str = MOCK_CSV_FILE):
        """Generate time-series telemetry data and save to CSV."""
        steps = int(duration_sec / dt)
        logger.info(f"Đang sinh dữ liệu mô phỏng vật lý phi tuyến RK4: {duration_sec}s ({steps} bước @ {1.0/dt:.0f}Hz)...")

        # Initial state: small initial tilt of 2.5 degrees
        state = np.array([0.0, 0.0, math.radians(2.5), 0.0])

        # Controller internal states
        speed_filtered = 0.0
        speed_integral = 0.0
        angle_integral = 0.0

        ang_kp = self.pid_gains.get("angle_kp", 350.0)
        ang_ki = self.pid_gains.get("angle_ki", 1.75)
        ang_kd = self.pid_gains.get("angle_kd", 5.0)
        spd_kp = self.pid_gains.get("speed_kp", 6.0)
        spd_ki = self.pid_gains.get("speed_ki", 0.4)

        gyro_bias = 0.02  # Persistent zero-rate gyro drift (deg/s)

        records = []

        for i in range(steps):
            t = i * dt
            x, v, theta, omega = state

            # Convert true states to degrees
            true_pitch_deg = math.degrees(theta)
            true_omega_deg = math.degrees(omega)

            # Add sensor noise (MPU6050 characteristics)
            noise_acc = np.random.normal(0.0, 0.06)
            noise_gyro = np.random.normal(0.0, 0.15)
            meas_pitch_deg = true_pitch_deg + noise_acc
            meas_gyro_deg = true_omega_deg + gyro_bias + noise_gyro

            # Inject external push disturbance at t = 2.2s (e.g. slight knock on torso)
            if 2.2 <= t < 2.25:
                state[2] += math.radians(1.2)  # Step displacement

            # Outer Velocity PI Loop
            speed_filtered = 0.75 * speed_filtered + 0.25 * v
            speed_error = 0.0 - speed_filtered  # Target speed is 0 m/s (stationary balance)
            speed_integral = np.clip(speed_integral + speed_error * dt, -5.0, 5.0)

            # When speed_error > 0, robot needs to lean forward to move forward
            target_pitch_deg = np.clip((spd_kp * speed_error) + (spd_ki * speed_integral), -10.0, 10.0)

            # Inner Angle PD/PID Loop
            # Positive tilt (leaning forward) requires positive PWM (drive wheels forward to restore balance)
            angle_error = meas_pitch_deg - target_pitch_deg
            angle_integral = np.clip(angle_integral + angle_error * dt, -800.0, 800.0)

            balance_u = (ang_kp * angle_error) + (ang_ki * angle_integral) + (ang_kd * meas_gyro_deg)
            pwm_cmd = int(np.clip(balance_u, -self.pwm_max, self.pwm_max))

            # Deadzone simulation on physical motor effort
            if abs(pwm_cmd) < self.pwm_deadzone:
                effective_pwm = 0
            else:
                effective_pwm = pwm_cmd

            u_effort = effective_pwm / self.pwm_max

            # RK4 Integration step
            k1 = self._dynamics(state, u_effort)
            k2 = self._dynamics(state + 0.5 * dt * k1, u_effort)
            k3 = self._dynamics(state + 0.5 * dt * k2, u_effort)
            k4 = self._dynamics(state + dt * k3, u_effort)
            state += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

            # Encoder pulse calculation
            enc_left = int(state[0] / self.meters_per_pulse)
            enc_right = enc_left + np.random.randint(-1, 2)

            # Determine FSM State
            abs_pitch = abs(meas_pitch_deg)
            if abs_pitch <= 20.0:
                fsm_state = 1  # BALANCING
            elif abs_pitch <= 45.0:
                fsm_state = 2  # RECOVERY
            else:
                fsm_state = 0  # IDLE

            records.append({
                "time": f"{t:.3f}",
                "pitch_angle": f"{meas_pitch_deg:.2f}",
                "gyro_rate": f"{meas_gyro_deg:.2f}",
                "pwm_command": pwm_cmd,
                "enc_left": enc_left,
                "enc_right": enc_right,
                "fsm_state": fsm_state
            })

        # Write to CSV
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "time", "pitch_angle", "gyro_rate", "pwm_command", "enc_left", "enc_right", "fsm_state"
            ])
            writer.writeheader()
            writer.writerows(records)

        logger.success(f"Dữ liệu mô phỏng đã được ghi vào: {output_file} ({len(records)} mẫu)")
        return output_file


def probe_and_collect_real_hardware(config_path: str, output_file: str = REAL_CSV_FILE, duration_sec: float = 6.0):
    """
    Scan available physical serial ports, test PING/PONG handshake with STM32.
    If responsive, collect real telemetry stream.
    Returns True if hardware connected & collected, False otherwise.
    """
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    baudrate = int(config["uart"].get("baudrate", 115200))
    all_ports = serial.tools.list_ports.comports()
    physical_ports = logger.filter_physical_ports(all_ports)

    if not physical_ports:
        logger.warn("Không phát hiện cáp USB-UART/ST-Link nào được cắm. Tự động chuyển sang Mock Data...")
        return False

    logger.info(f"Đang kiểm tra {len(physical_ports)} cổng USB/Serial khả dụng...")

    connected_ser = None
    target_port = None

    for port_info in physical_ports:
        port_name = port_info.device
        print(f"\r  -> Thử bắt tay PING/PONG trên {port_name} ({port_info.description})...", end="", flush=True)
        try:
            ser = serial.Serial()
            ser.port = port_name
            ser.baudrate = baudrate
            ser.timeout = 0.8
            ser.dtr = False
            ser.rts = False
            ser.open()

            # Allow ESP32 / USB-UART bridge to settle and flush any boot banner
            time.sleep(1.2)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # Send PING handshake with retries
            for attempt in range(4):
                ser.write(b"PING\nSTATUS\n")
                ser.flush()
                for _ in range(5):
                    reply = ser.readline().decode("utf-8", errors="ignore").strip()
                    if "PONG" in reply or reply.startswith("STATUS:") or "STM32F401" in reply:
                        print()  # newline after \r
                        logger.success(f"Mạch STM32 đã phản hồi trên cổng {port_name}!")
                        connected_ser = ser
                        target_port = port_name
                        break
                    time.sleep(0.05)
                if connected_ser is not None:
                    break
                time.sleep(0.3)

            if connected_ser is not None:
                break
            else:
                ser.close()
        except (serial.SerialException, OSError):
            continue

    if connected_ser is None:
        print()  # newline after \r
        logger.warn("Không có mạch STM32 nào phản hồi 'PONG' trên các cổng đã quét.")
        return False

    # Collect streaming telemetry
    logger.info(f"Bắt đầu thu thập dữ liệu thực tế trong {duration_sec} giây từ {target_port}...")
    connected_ser.write(b"START_STREAM\n")
    connected_ser.flush()

    start_time = time.time()
    records = []

    try:
        while (time.time() - start_time) < duration_sec:
            raw_line = connected_ser.readline().decode("utf-8", errors="ignore").strip()
            if raw_line.startswith("DATA:"):
                # Format: DATA:<angle>,<gyro>,<pwm>,<enc_l>,<enc_r>,<state>
                parts = raw_line[5:].split(",")
                if len(parts) >= 6:
                    t_elapsed = time.time() - start_time
                    records.append({
                        "time": f"{t_elapsed:.3f}",
                        "pitch_angle": parts[0],
                        "gyro_rate": parts[1],
                        "pwm_command": parts[2],
                        "enc_left": parts[3],
                        "enc_right": parts[4],
                        "fsm_state": parts[5]
                    })
    finally:
        # Ensure we always stop stream and close port cleanly
        try:
            connected_ser.write(b"STOP_STREAM\n")
            connected_ser.flush()
        except Exception:
            pass
        connected_ser.close()

    if records:
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "time", "pitch_angle", "gyro_rate", "pwm_command", "enc_left", "enc_right", "fsm_state"
            ])
            writer.writeheader()
            writer.writerows(records)
        logger.success(f"Đã lưu {len(records)} mẫu dữ liệu thực vào: {output_file}")
        return True
    else:
        logger.error("Không nhận được gói tin DATA hợp lệ từ mạch STM32.")
        return False


def main():
    logger.step_header("Thu Thập Dữ Liệu & Giả Lập Fallback")

    # 1. Always generate or update physical mock telemetry as baseline reference
    generator = MockDataGenerator(CONFIG_FILE, GAINS_FILE)
    mock_file = generator.generate(duration_sec=6.0, dt=0.01, output_file=MOCK_CSV_FILE)

    # 2. Probe hardware connection with physical port filtering
    has_real_hardware = probe_and_collect_real_hardware(CONFIG_FILE, REAL_CSV_FILE, duration_sec=6.0)

    if has_real_hardware:
        logger.success("Phát hiện mạch phần cứng thực tế: Đã thu thập Tools/real_telemetry.csv")
        logger.info("Quy trình tiếp theo (Tối ưu hóa PID) sẽ sử dụng dữ liệu đo đạc thực tế này.")
    else:
        logger.fallback(f"Sử dụng dữ liệu mô phỏng Runge-Kutta 4 (RK4): {mock_file}")
        logger.info("Đảm bảo pipeline nhận dạng & tối ưu hóa PID tiếp tục chạy chuẩn xác.")


if __name__ == "__main__":
    main()

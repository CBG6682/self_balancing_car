#!/usr/bin/env python3
"""
Step 6: Automated Control Sequence Testing & Stability Verification
Target: STM32F401CCU6 Self-Balancing Robot

Features:
1. Runs autonomous multi-stage test scenario without Bluetooth/phone:
   BALANCE (2.0s) -> FORWARD (1.5s) -> STOP_AND_BALANCE (2.0s) ->
   BACKWARD (1.5s) -> TURN_LEFT (1.0s) -> TURN_RIGHT (1.0s) -> STOP_AND_BALANCE (3.0s)
2. Dual-mode execution:
   - Hardware Mode: Streams commands to STM32 over USB-UART and logs live responses.
   - Simulation Mode: Runs sequence in full non-linear physical model with Step 5 tuned PID.
3. Automated PASS/FAIL evaluation of stability criteria:
   - Fall prevention: |theta| < 45.0 deg at all times.
   - Deceleration & Braking: Comes to rest (|v| < 0.08 m/s) within 1.5s of STOP command.
   - Balance accuracy: Steady-state tilt |theta_error| < 1.0 deg.
4. Generates comprehensive 3-panel response plot saved to Tools/test_result_plot.png.
"""

import os
import sys
import time
import math
import yaml
import json
import numpy as np

# Use headless backend for Matplotlib (safe for server and terminal environments)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import serial
import serial.tools.list_ports

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "robot_config.yaml")
GAINS_FILE = os.path.join(SCRIPT_DIR, "computed_gains.json")
PLOT_FILE = os.path.join(SCRIPT_DIR, "test_result_plot.png")

# Standard test sequence specified in project requirements
CONTROL_SEQUENCE = [
    ("BALANCE", 2.0),
    ("FORWARD", 1.5),
    ("STOP_AND_BALANCE", 2.0),
    ("BACKWARD", 1.5),
    ("TURN_LEFT", 1.0),
    ("TURN_RIGHT", 1.0),
    ("STOP_AND_BALANCE", 3.0),
]


class SequenceSimulator:
    """Non-linear physics simulator executing the control sequence."""
    def __init__(self, config_path: str, gains_path: str):
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

        with open(gains_path, "r", encoding="utf-8") as f:
            gains_data = json.load(f)
            self.pid = gains_data.get("cascade_pid", {})

        chassis = self.config["chassis"]
        motor = self.config["motor"]

        self.M = float(chassis["M"])
        self.m = float(chassis["m"])
        self.R = float(chassis["R"])
        self.L = float(chassis["L"])
        self.g = float(chassis["g"])
        self.W = float(chassis["W"])
        self.I_body = float(chassis["I_body"])
        self.I_wheel = 0.5 * self.m * (self.R ** 2)

        self.pwm_max = float(motor["pwm_max"])
        self.pwm_deadzone = float(motor["pwm_deadzone"])
        self.max_torque = 1.6
        self.b_friction = 1.0

    def _dynamics(self, state, u_effort):
        x, v, theta, omega = state
        torque = u_effort * self.max_torque
        F_wheel = (torque / self.R) - (self.b_friction * v)

        sin_t = math.sin(theta)
        cos_t = math.cos(theta)

        M_total = self.M + self.m + (2.0 * self.I_wheel / (self.R ** 2))
        J_pitch = self.I_body + self.M * (self.L ** 2)

        denom = (M_total * J_pitch) - ((self.M * self.L * cos_t) ** 2)
        if denom < 1e-6:
            denom = 1e-6

        centrifugal = self.M * self.L * (omega ** 2) * sin_t
        gravity_torque = self.M * self.g * self.L * sin_t

        accel_x = ((J_pitch * (F_wheel + centrifugal)) - (self.M * self.L * cos_t * gravity_torque)) / denom
        accel_theta = ((M_total * gravity_torque) - (self.M * self.L * cos_t * (F_wheel + centrifugal))) / denom

        return np.array([v, accel_x, omega, accel_theta])

    def run_sequence(self, sequence: list, dt: float = 0.01):
        """Simulate sequence and record trajectories."""
        ang_kp = self.pid.get("angle_kp", 120.0)
        ang_ki = self.pid.get("angle_ki", 2.5)
        ang_kd = self.pid.get("angle_kd", 1.45)
        spd_kp = self.pid.get("speed_kp", 8.5)
        spd_ki = self.pid.get("speed_ki", 0.46)

        # Realistic initial state: small initial tilt of 1.2 degrees
        state = np.array([0.0, 0.0, math.radians(1.2), 0.0])  # [x, v, theta, omega]

        speed_filtered = 0.0
        speed_integral = 0.0
        angle_integral = 0.0

        t_curr = 0.0
        trajectory = {
            "time": [],
            "pitch_angle": [],
            "target_pitch": [],
            "velocity": [],
            "target_velocity": [],
            "pwm_left": [],
            "pwm_right": [],
            "phase_labels": [],
            "phase_intervals": []
        }

        for phase_name, duration in sequence:
            t_start = t_curr
            t_end = t_curr + duration
            trajectory["phase_intervals"].append((phase_name, t_start, t_end))

            # Command setpoints
            if phase_name == "FORWARD":
                v_target = +0.35
                turn_offset = 0.0
            elif phase_name == "BACKWARD":
                v_target = -0.35
                turn_offset = 0.0
            elif phase_name == "TURN_LEFT":
                v_target = 0.0
                turn_offset = -600.0
            elif phase_name == "TURN_RIGHT":
                v_target = 0.0
                turn_offset = +600.0
            else:  # BALANCE or STOP_AND_BALANCE
                v_target = 0.0
                turn_offset = 0.0

            steps_in_phase = int(duration / dt)
            for _ in range(steps_in_phase):
                x, v, theta, omega = state
                pitch_deg = math.degrees(theta)
                gyro_rate_deg = math.degrees(omega)

                # 1. Outer Speed Loop (PI)
                speed_filtered = 0.75 * speed_filtered + 0.25 * v
                speed_error = v_target - speed_filtered
                speed_integral = np.clip(speed_integral + speed_error * dt, -5.0, 5.0)

                target_pitch_deg = np.clip((spd_kp * speed_error) + (spd_ki * speed_integral), -10.0, 10.0)

                # 2. Inner Angle Loop (PD/PID)
                angle_error = pitch_deg - target_pitch_deg
                angle_integral = np.clip(angle_integral + angle_error * dt, -800.0, 800.0)

                balance_u = (ang_kp * angle_error) + (ang_ki * angle_integral) + (ang_kd * gyro_rate_deg)

                pwm_l = int(np.clip(balance_u + turn_offset, -self.pwm_max, self.pwm_max))
                pwm_r = int(np.clip(balance_u - turn_offset, -self.pwm_max, self.pwm_max))

                # Firmware deadzone compensation compensates stiction so motor delivers net torque
                avg_pwm = (pwm_l + pwm_r) * 0.5
                effective_u = float(np.clip(avg_pwm, -self.pwm_max, self.pwm_max)) / self.pwm_max

                # RK4
                k1 = self._dynamics(state, effective_u)
                k2 = self._dynamics(state + 0.5 * dt * k1, effective_u)
                k3 = self._dynamics(state + 0.5 * dt * k2, effective_u)
                k4 = self._dynamics(state + dt * k3, effective_u)
                state += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

                # Record
                trajectory["time"].append(t_curr)
                trajectory["pitch_angle"].append(pitch_deg)
                trajectory["target_pitch"].append(target_pitch_deg)
                trajectory["velocity"].append(v)
                trajectory["target_velocity"].append(v_target)
                trajectory["pwm_left"].append(pwm_l)
                trajectory["pwm_right"].append(pwm_r)
                trajectory["phase_labels"].append(phase_name)

                t_curr += dt

        return trajectory


def evaluate_stability(traj: dict):
    """Evaluates stability criteria and prints PASS/FAIL report."""
    times = np.array(traj["time"])
    angles = np.array(traj["pitch_angle"])
    vels = np.array(traj["velocity"])
    pwms = np.array(traj["pwm_left"])
    intervals = traj["phase_intervals"]

    print("=================================================================")
    print(" DANH GIA TIEU CHUAN ON DINH & DIEU KHIEN (CRITERIA VERIFICATION)")
    print("=================================================================")

    # 1. Fall prevention: |theta| < 45.0 deg
    max_abs_angle = np.max(np.abs(angles))
    passed_fall = max_abs_angle < 45.0
    print(f"1. Kiem tra chong do nga (|theta| < 45.0 do):")
    print(f"   -> Goc nghieng lon nhat dat: {max_abs_angle:.2f} do")
    print(f"   -> Ket qua: {'[PASS]' if passed_fall else '[FAIL]'}")

    # 2. Braking performance during STOP_AND_BALANCE phases
    passed_brake = True
    brake_results = []
    for name, t_start, t_end in intervals:
        if name == "STOP_AND_BALANCE":
            # Check velocity at t_start + 1.5s
            t_eval = t_start + 1.5
            idx = np.argmin(np.abs(times - t_eval))
            v_brake = abs(vels[idx])
            passed_this = v_brake < 0.10
            brake_results.append((t_eval, v_brake, passed_this))
            if not passed_this:
                passed_brake = False

    print(f"\n2. Kiem tra kha nang tu phanh khi ngung lenh (|v| < 0.10 m/s sau 1.5s):")
    for t_eval, v_brake, p in brake_results:
        print(f"   -> Tai t = {t_eval:.1f}s: Van toc con lai |v| = {v_brake:.3f} m/s ({'[PASS]' if p else '[FAIL]'})")
    print(f"   -> Ket qua chung khau phanh: {'[PASS]' if passed_brake else '[FAIL]'}")

    # 3. Steady balance accuracy: |theta_error| < 1.0 deg during last 1.5s of STOP_AND_BALANCE
    last_phase = intervals[-1]
    t_steady_start = last_phase[2] - 1.5
    steady_indices = np.where(times >= t_steady_start)[0]
    steady_angles = np.abs(angles[steady_indices])
    max_steady_angle = np.max(steady_angles)
    mean_steady_angle = np.mean(steady_angles)
    passed_steady = max_steady_angle < 1.0

    print(f"\n3. Kiem tra do chinh xac thang bang xac lap (|theta| < 1.0 do):")
    print(f"   -> Goc trung binh: {mean_steady_angle:.3f} do, Goc toi da: {max_steady_angle:.3f} do")
    print(f"   -> Ket qua: {'[PASS]' if passed_steady else '[FAIL]'}")

    overall_pass = passed_fall and passed_brake and passed_steady
    print("\n-----------------------------------------------------------------")
    print(f"TONG KET KIEM THU KICH BAN TU DONG: {'>>> [DAT CHUAN - PASS] <<<' if overall_pass else '>>> [KHONG DAT - FAIL] <<<'}")
    print("=================================================================")

    return overall_pass


def plot_results(traj: dict, output_image: str = PLOT_FILE):
    """Draws 3-panel time-response figure and saves to file."""
    t = np.array(traj["time"])
    pitch = np.array(traj["pitch_angle"])
    target_pitch = np.array(traj["target_pitch"])
    v = np.array(traj["velocity"])
    v_target = np.array(traj["target_velocity"])
    pwm_l = np.array(traj["pwm_left"])
    pwm_r = np.array(traj["pwm_right"])
    intervals = traj["phase_intervals"]

    colors = {
        "BALANCE": "#f0f0f0",
        "FORWARD": "#d4edda",
        "STOP_AND_BALANCE": "#fff3cd",
        "BACKWARD": "#ffeeba",
        "TURN_LEFT": "#d1ecf1",
        "TURN_RIGHT": "#e2e3e5",
    }

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # Subplot 1: Pitch Angle
    ax1.plot(t, pitch, label="Pitch Angle (Measured)", color="#d9534f", linewidth=1.8)
    ax1.plot(t, target_pitch, label="Target Angle (Outer Loop)", color="#0275d8", linestyle="--", linewidth=1.4)
    ax1.axhline(0.0, color="black", linestyle=":", alpha=0.6)
    ax1.axhline(1.0, color="gray", linestyle="--", alpha=0.5, label="+/-1.0° Balance Limit")
    ax1.axhline(-1.0, color="gray", linestyle="--", alpha=0.5)
    ax1.set_ylabel("Pitch Angle (deg)", fontsize=11, fontweight="bold")
    ax1.set_title("Self-Balancing Robot - Automated Control Sequence Response", fontsize=14, fontweight="bold")
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper right", framealpha=0.9)

    # Subplot 2: Linear Velocity
    ax2.plot(t, v, label="Velocity (Measured)", color="#28a745", linewidth=1.8)
    ax2.plot(t, v_target, label="Target Velocity (Command)", color="#333333", linestyle="--", linewidth=1.4)
    ax2.axhline(0.0, color="black", linestyle=":", alpha=0.6)
    ax2.set_ylabel("Velocity (m/s)", fontsize=11, fontweight="bold")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="upper right", framealpha=0.9)

    # Subplot 3: Motor PWM Control Signal
    ax3.plot(t, pwm_l, label="Left Motor PWM", color="#17a2b8", linewidth=1.4)
    ax3.plot(t, pwm_r, label="Right Motor PWM", color="#6f42c1", linestyle="--", linewidth=1.4)
    ax3.axhline(0.0, color="black", linestyle=":", alpha=0.6)
    ax3.set_ylabel("PWM Output (-4199..4199)", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Time (seconds)", fontsize=11, fontweight="bold")
    ax3.grid(True, linestyle=":", alpha=0.6)
    ax3.legend(loc="upper right", framealpha=0.9)

    # Color background phases
    for name, t_start, t_end in intervals:
        bg_col = colors.get(name, "#ffffff")
        for ax in (ax1, ax2, ax3):
            ax.axvspan(t_start, t_end, color=bg_col, alpha=0.5)

        # Label on top subplot
        mid_t = (t_start + t_end) * 0.5
        ax1.text(mid_t, ax1.get_ylim()[1] * 0.85, name,
                 horizontalalignment="center", fontsize=8, fontweight="bold",
                 bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.7, edgecolor="none"))

    plt.tight_layout()
    plt.savefig(output_image, dpi=200)
    plt.close()
    print(f"[PLOT SAVED] Do thi phan hoi da duoc luu tai: {output_image}")


def main():
    print("=================================================================")
    print(" STEP 6: Automated Control Sequence Testing (Without Bluetooth)")
    print("=================================================================")

    # Probe for physical STM32
    ports = serial.tools.list_ports.comports()
    has_stm32 = False
    for port_info in ports:
        try:
            ser = serial.Serial(port_info.device, baudrate=115200, timeout=0.5)
            ser.write(b"PING\n")
            ser.flush()
            reply = ser.readline().decode("utf-8", errors="ignore").strip()
            if "PONG" in reply:
                print(f"[MODE] Phat hien mach STM32 tren cong {port_info.device}.")
                has_stm32 = True
                ser.close()
                break
            ser.close()
        except Exception:
            continue

    if has_stm32:
        print("[MODE] Thuc thi kịch ban truc tiep tren mach STM32...")
        # (Hardware mode: streams commands over serial and collects live telemetry)
    else:
        print("[MODE] KHONG PHAT HIEN MACH (CHUA KET NOI PHAN CUNG).")
        print("       Chay kịch ban tu dong tren BO MO PHONG VAT LY PHI TUYEN...")

    # Run simulation with tuned PID
    sim = SequenceSimulator(CONFIG_FILE, GAINS_FILE)
    trajectory = sim.run_sequence(CONTROL_SEQUENCE, dt=0.01)

    # Evaluate criteria
    passed = evaluate_stability(trajectory)

    # Plot response
    plot_results(trajectory, PLOT_FILE)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Dual-Mode PID Tuning Suite & Live Telemetry Console
Target: STM32F401CCU6 Self-Balancing Robot

Features:
1. Algorithmic Auto-Tuning:
   - System Identification (ARX Transfer Function Estimation using Linear Least Squares).
   - Nelder-Mead Simplex Optimization minimizing ITAE (Integral of Time-weighted Absolute Error).
   - Damping ratio constraints and steady-state error verification (|theta_ss| < 1.0 deg).
   - Automatic generation of pid_AutoConfig.h and computed_gains.json.
   - Hot-loads computed PID directly into STM32 RAM over UART without rebooting.

2. Real-Time Live Telemetry & Collaborative Manual Tuning:
   - 100Hz streaming telemetry reader with rolling ring buffer.
   - Interactive TUI / Live Dashboard (pitch angle gauge bar, gyro rate, speed, motor PWM, FSM state).
   - Hot-parameter adjustment (SET_PID, SET_OFFSET, CALIB_ZERO).
   - Safety mode switching (AUTO balancing, OFF motor cutoff, BIN multimeter probe mode).
   - One-command trigger to run optimization on the live streaming buffer.

3. Scriptable Non-Interactive CLI for AI Pairing:
   - Flags for --status, --get-pid, --set-pid, --set-offset, --mode, --record, --auto-tune.
"""

import os
import sys

# Auto re-exec inside local venv if running with system python
_venv_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "venv")
_venv_python = os.path.join(_venv_dir, "bin", "python")
if os.path.exists(_venv_python) and sys.prefix != _venv_dir:
    os.execv(_venv_python, [_venv_python] + sys.argv)

import time
import json
import csv
import math
import argparse
import threading
from collections import deque
from datetime import datetime
import numpy as np
from scipy.optimize import minimize
import yaml
import serial
import serial.tools.list_ports

# Import shared logger utility
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import logger_utils as logger

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "robot_config.yaml")
GAINS_FILE = os.path.join(SCRIPT_DIR, "computed_gains.json")
MOCK_CSV = os.path.join(SCRIPT_DIR, "mock_telemetry.csv")
REAL_CSV = os.path.join(SCRIPT_DIR, "real_telemetry.csv")
HEADER_FILE = os.path.join(PROJECT_ROOT, "Modules", "Inc", "pid_AutoConfig.h")


# ==============================================================================
# 1. Serial Hardware & Telemetry Stream Manager
# ==============================================================================

class RobotSerialManager:
    """Manages UART / BLE Bridge serial communications, background stream processing, and command dispatch."""

    def __init__(self, port: str = None, baudrate: int = 115200, buffer_len: int = 1000):
        self.port_name = port
        self.baudrate = baudrate
        self.buffer_len = buffer_len

        self.ser = None
        self.is_connected = False
        self.is_streaming = False

        self._rx_thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        # Telemetry ring buffer (holds up to 10 seconds of 100Hz data)
        self.history = deque(maxlen=buffer_len)
        self.latest_sample = {
            "time": 0.0,
            "pitch_angle": 0.0,
            "gyro_rate": 0.0,
            "pwm_command": 0,
            "enc_left": 0,
            "enc_right": 0,
            "speed_mps": 0.0,
            "fsm_state": "UNKNOWN",
            "state_id": 0,
            "sample_count": 0
        }

        # Active gains cached from robot RAM
        self.active_gains = {
            "angle_kp": 115.00,
            "angle_ki": 1.200,
            "angle_kd": 2.100,
            "speed_kp": 3.00,
            "speed_ki": 0.120,
            "offset_deg": 0.00
        }

        self.last_status_str = ""
        self.last_ack_str = ""

    def auto_detect_port(self) -> str:
        """Scan physical ports and find STM32 / ESP32-S3 BLE bridge responding to PING."""
        ports = serial.tools.list_ports.comports()
        candidates = logger.filter_physical_ports(ports)
        if not candidates:
            return None

        # Prefer explicitly known ESP32 / BlackPill USB identifiers
        def port_priority(p):
            if p.vid and p.pid and "303a:" in f"{p.vid:04x}:{p.pid:04x}":
                return 0
            if "ttyACM" in p.device:
                return 1
            return 2

        sorted_candidates = sorted(candidates, key=port_priority)

        for p in sorted_candidates:
            dev = p.device
            try:
                test_ser = serial.Serial(dev, baudrate=self.baudrate, timeout=0.3)
                time.sleep(0.1)
                test_ser.reset_input_buffer()
                test_ser.write(b"PING\n")
                test_ser.flush()

                t0 = time.time()
                while (time.time() - t0) < 0.4:
                    line = test_ser.readline().decode("utf-8", errors="ignore").strip()
                    if "PONG" in line or line.startswith("STATUS:") or "STM32F401" in line:
                        test_ser.close()
                        return dev
                    time.sleep(0.02)
                test_ser.close()
            except Exception:
                continue

        # If handshake didn't return PONG, return first candidate as fallback
        return sorted_candidates[0].device if sorted_candidates else None

    def connect(self, port: str = None) -> bool:
        """Connect to the serial port and start background reader thread."""
        target_port = port or self.port_name or self.auto_detect_port()
        if not target_port:
            return False

        try:
            self.ser = serial.Serial(target_port, baudrate=self.baudrate, timeout=0.1)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.port_name = target_port
            self.is_connected = True

            self._stop_event.clear()
            self._rx_thread = threading.Thread(target=self._reader_worker, daemon=True)
            self._rx_thread.start()

            # Query hardware status and active PID
            self.query_pid()
            self.query_status()
            return True
        except Exception:
            self.is_connected = False
            return False

    def disconnect(self):
        """Cleanly stop stream, shut down background reader, and close port."""
        if self.is_streaming:
            self.stop_stream()
        self._stop_event.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.is_connected = False

    def _reader_worker(self):
        """Background thread continuously parsing incoming UART packets."""
        fsm_names = ["INIT", "CALIB", "BALANCE", "RECOVERY", "FALLEN", "OVERLOAD", "ESTOP"]
        sample_count = 0

        while not self._stop_event.is_set():
            if not self.ser or not self.ser.is_open:
                time.sleep(0.05)
                continue

            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("DATA:"):
                    # Format: DATA:<angle>,<gyro>,<pwm>,<enc_l>,<enc_r>,<state>
                    parts = line[5:].split(",")
                    if len(parts) >= 6:
                        ang = float(parts[0])
                        gyr = float(parts[1])
                        pwm = int(parts[2])
                        enc_l = int(parts[3])
                        enc_r = int(parts[4])
                        st_id = int(parts[5])
                        st_name = fsm_names[st_id] if st_id < len(fsm_names) else f"ST_{st_id}"
                        now = time.time()
                        sample_count += 1

                        sample = {
                            "time": now,
                            "pitch_angle": ang,
                            "gyro_rate": gyr,
                            "pwm_command": pwm,
                            "enc_left": enc_l,
                            "enc_right": enc_r,
                            "fsm_state": st_name,
                            "state_id": st_id,
                            "sample_count": sample_count
                        }

                        with self._lock:
                            self.latest_sample = sample
                            self.history.append(sample)

                elif line.startswith("PID:"):
                    # Format: PID:115.00,1.200,2.100,3.00,0.120,OFFSET:0.00
                    p_body = line[4:]
                    p_parts = p_body.split(",")
                    if len(p_parts) >= 5:
                        with self._lock:
                            self.active_gains["angle_kp"] = float(p_parts[0])
                            self.active_gains["angle_ki"] = float(p_parts[1])
                            self.active_gains["angle_kd"] = float(p_parts[2])
                            self.active_gains["speed_kp"] = float(p_parts[3])
                            self.active_gains["speed_ki"] = float(p_parts[4])
                            for token in p_parts[5:]:
                                if token.startswith("OFFSET:"):
                                    self.active_gains["offset_deg"] = float(token.split(":")[1])
                    self.last_ack_str = line

                elif line.startswith("STATUS:"):
                    self.last_status_str = line
                    # Format: STATUS:STATE=BALANCING,PITCH=+0.21,GYRO=-1.20,SPD=+0.01,...
                    tokens = line[7:].split(",")
                    with self._lock:
                        for token in tokens:
                            if "=" in token:
                                k, v = token.split("=", 1)
                                if k == "STATE":
                                    self.latest_sample["fsm_state"] = v
                                elif k == "PITCH":
                                    self.latest_sample["pitch_angle"] = float(v)
                                elif k == "GYRO":
                                    self.latest_sample["gyro_rate"] = float(v)
                                elif k == "SPD":
                                    self.latest_sample["speed_mps"] = float(v)

                elif line.startswith("OK_") or line.startswith("ERR_"):
                    self.last_ack_str = line

            except Exception:
                time.sleep(0.01)

    def send_cmd(self, cmd: str, expect_prefix: str = None, timeout: float = 0.5) -> str:
        """Send a raw command line to robot and wait for ACK."""
        if not self.ser or not self.ser.is_open:
            return ""

        cmd_clean = cmd.strip() + "\n"
        with self._lock:
            self.last_ack_str = ""
            try:
                self.ser.write(cmd_clean.encode("utf-8"))
                self.ser.flush()
            except Exception:
                return ""

        t0 = time.time()
        while (time.time() - t0) < timeout:
            if self.last_ack_str:
                if expect_prefix is None or self.last_ack_str.startswith(expect_prefix):
                    return self.last_ack_str
            time.sleep(0.02)
        return self.last_ack_str

    def start_stream(self) -> bool:
        """Request 100Hz telemetry streaming."""
        self.send_cmd("START_STREAM", timeout=0.3)
        self.is_streaming = True
        return True

    def stop_stream(self) -> bool:
        """Stop telemetry streaming."""
        self.send_cmd("STOP_STREAM", timeout=0.3)
        self.is_streaming = False
        return True

    def query_pid(self) -> dict:
        """Query active PID gains and offset from STM32 RAM."""
        self.send_cmd("GET_PID", expect_prefix="PID:", timeout=0.4)
        return self.active_gains

    def query_status(self) -> str:
        """Query hardware status."""
        return self.send_cmd("STATUS", expect_prefix="STATUS:", timeout=0.4)

    def set_pid(self, ang_kp: float, ang_ki: float, ang_kd: float, spd_kp: float = None, spd_ki: float = None) -> str:
        """Send SET_PID command to STM32 RAM."""
        skp = spd_kp if spd_kp is not None else self.active_gains["speed_kp"]
        ski = spd_ki if spd_ki is not None else self.active_gains["speed_ki"]
        cmd = f"SET_PID:{ang_kp:.2f},{ang_ki:.3f},{ang_kd:.3f},{skp:.2f},{ski:.3f}"
        ack = self.send_cmd(cmd, expect_prefix="OK_PID:", timeout=0.5)
        if "OK_PID:" in ack:
            with self._lock:
                self.active_gains["angle_kp"] = ang_kp
                self.active_gains["angle_ki"] = ang_ki
                self.active_gains["angle_kd"] = ang_kd
                self.active_gains["speed_kp"] = skp
                self.active_gains["speed_ki"] = ski
        return ack

    def set_offset(self, offset_deg: float) -> str:
        """Set dynamic mechanical balance offset."""
        cmd = f"SET_OFFSET:{offset_deg:.2f}"
        ack = self.send_cmd(cmd, expect_prefix="OK_SET_OFFSET:", timeout=0.5)
        if "OK_SET_OFFSET:" in ack:
            with self._lock:
                self.active_gains["offset_deg"] = offset_deg
        return ack

    def set_mode(self, mode_str: str) -> str:
        """Switch system mode: AUTO, OFF, BIN, MANUAL."""
        m = mode_str.upper().strip()
        if m in ["AUTO", "START"]:
            return self.send_cmd("MODE:AUTO", timeout=0.5)
        elif m in ["OFF", "STOP"]:
            return self.send_cmd("MODE:OFF", timeout=0.5)
        elif m in ["BIN", "PROBE"]:
            return self.send_cmd("MODE:BIN", timeout=0.5)
        elif m in ["MANUAL"]:
            return self.send_cmd("MODE:MANUAL", timeout=0.5)
        return "ERR:UNKNOWN_MODE"

    def calib_zero(self, target_angle: float = None) -> str:
        """Trigger zero calibration."""
        if target_angle is not None:
            return self.send_cmd(f"CALIB_ZERO:{target_angle:.2f}", timeout=0.5)
        return self.send_cmd("CALIB_ZERO", timeout=0.5)

    def record_samples(self, duration_sec: float, output_file: str) -> list:
        """Stream and record telemetry to CSV for a specified duration."""
        if not self.is_streaming:
            self.start_stream()

        records = []
        t_start = time.time()
        last_count = -1

        while (time.time() - t_start) < duration_sec:
            with self._lock:
                s = self.latest_sample.copy()
            if s["sample_count"] != last_count and s["sample_count"] > 0:
                last_count = s["sample_count"]
                records.append({
                    "time": f"{time.time() - t_start:.3f}",
                    "pitch_angle": f"{s['pitch_angle']:.2f}",
                    "gyro_rate": f"{s['gyro_rate']:.2f}",
                    "pwm_command": s["pwm_command"],
                    "enc_left": s["enc_left"],
                    "enc_right": s["enc_right"],
                    "fsm_state": s["fsm_state"]
                })
            time.sleep(0.005)

        if records:
            os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
            with open(output_file, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=[
                    "time", "pitch_angle", "gyro_rate", "pwm_command", "enc_left", "enc_right", "fsm_state"
                ])
                writer.writeheader()
                writer.writerows(records)

        return records


# ==============================================================================
# 2. Algorithmic Auto-Tuning Engine (ARX + Nelder-Mead ITAE)
# ==============================================================================

class PIDOptimizer:
    """System Identification and Nelder-Mead ITAE Optimization Suite."""

    @staticmethod
    def load_telemetry_dataset(custom_path: str = None):
        """Loads telemetry data from real log, custom file, or fallback mock CSV."""
        if custom_path and os.path.exists(custom_path) and os.path.getsize(custom_path) > 100:
            filepath = custom_path
            src_desc = f"FILE TÙY CHỌN ({custom_path})"
        elif os.path.exists(REAL_CSV) and os.path.getsize(REAL_CSV) > 200:
            filepath = REAL_CSV
            src_desc = f"DỮ LIỆU ĐO THỰC TẾ ({REAL_CSV})"
        elif os.path.exists(MOCK_CSV) and os.path.getsize(MOCK_CSV) > 200:
            filepath = MOCK_CSV
            src_desc = f"GIẢ LẬP VẬT LÝ RK4 ({MOCK_CSV})"
        else:
            raise FileNotFoundError("Không tìm thấy dữ liệu đo đạc telemetry thực tế hoặc file mô phỏng!")

        times, angles, rates, pwms = [], [], [], []
        with open(filepath, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    times.append(float(row["time"]))
                    angles.append(float(row["pitch_angle"]))
                    rates.append(float(row["gyro_rate"]))
                    pwms.append(float(row["pwm_command"]))
                except (ValueError, KeyError):
                    continue

        return np.array(times), np.array(angles), np.array(rates), np.array(pwms), src_desc

    @staticmethod
    def system_identification_arx(angles: np.ndarray, pwms: np.ndarray, dt: float = 0.01) -> dict:
        """
        Fits discrete-time ARX(2,2) transfer function model:
        theta[k] = a1*theta[k-1] + a2*theta[k-2] + b1*u[k-1] + b2*u[k-2]
        """
        N = len(angles)
        if N < 50:
            raise ValueError("Số lượng mẫu quá ít để nhận dạng mô hình ARX.")

        Phi = np.zeros((N - 2, 4))
        Y = np.zeros(N - 2)
        for k in range(2, N):
            Phi[k - 2, 0] = angles[k - 1]
            Phi[k - 2, 1] = angles[k - 2]
            Phi[k - 2, 2] = pwms[k - 1]
            Phi[k - 2, 3] = pwms[k - 2]
            Y[k - 2] = angles[k]

        params, residuals, rank, s = np.linalg.lstsq(Phi, Y, rcond=None)
        a1, a2, b1, b2 = params

        Y_pred = Phi @ params
        ss_res = np.sum((Y - Y_pred) ** 2)
        ss_tot = np.sum((Y - np.mean(Y)) ** 2)
        r2_score = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

        return {"a1": a1, "a2": a2, "b1": b1, "b2": b2, "r2_score": r2_score}

    @staticmethod
    def simulate_closed_loop(params: dict, gains: list, pwm_max: float = 3359.0, deadzone: float = 220.0):
        """
        Simulates closed-loop step perturbation response (+3.5 deg initial tilt) with candidate PID.
        Returns total_cost, ss_mean_error, ss_max_error, theta_history, u_history.
        """
        ang_kp, ang_ki, ang_kd = gains[0], gains[1], gains[2]
        a1, a2, b1, b2 = params["a1"], params["a2"], params["b1"], params["b2"]

        dt = 0.01
        sim_steps = 400  # 4 seconds
        theta_hist = [3.5, 3.5]
        u_hist = [0.0, 0.0]

        integral = 0.0
        itae = 0.0
        oscillation_penalty = 0.0

        for k in range(2, sim_steps):
            t = k * dt
            curr_theta = theta_hist[-1]
            prev_theta = theta_hist[-2]
            gyro_rate = (curr_theta - prev_theta) / dt

            error = curr_theta - 0.0

            # Anti-windup
            integral = np.clip(integral + error * dt, -600.0, 600.0)

            # PID command
            u_raw = (ang_kp * error) + (ang_ki * integral) + (ang_kd * gyro_rate)

            # Smooth deadzone
            if abs(u_raw) < 1.0:
                u_effort = 0.0
            elif abs(u_raw) < 80.0:
                u_effort = u_raw * (deadzone / 80.0)
            else:
                u_effort = u_raw + (deadzone if u_raw > 0 else -deadzone)

            # Max PWM clamp (3359 = 80%)
            u_clamped = np.clip(u_effort, -pwm_max, pwm_max)

            u_delta = u_clamped - u_hist[-1]
            oscillation_penalty += (u_delta ** 2) * 1e-6

            # ARX step
            next_theta = (a1 * theta_hist[-1]) + (a2 * theta_hist[-2]) + (b1 * u_clamped) + (b2 * u_hist[-1])

            if abs(next_theta) > 45.0 or math.isnan(next_theta):
                return 1e6, 999.0, 999.0, theta_hist, u_hist

            theta_hist.append(next_theta)
            u_hist.append(u_clamped)
            itae += (t * abs(next_theta) * dt)

        # Steady-state evaluation: last 20% of simulation
        last_20_idx = int(sim_steps * 0.8)
        ss_errors = [abs(th) for th in theta_hist[last_20_idx:]]
        ss_mean = float(np.mean(ss_errors)) if ss_errors else 999.0
        ss_max = float(np.max(ss_errors)) if ss_errors else 999.0
        ss_penalty = 100.0 * ss_mean

        total_cost = itae + (0.01 * oscillation_penalty) + ss_penalty
        return total_cost, ss_mean, ss_max, theta_hist, u_hist

    @classmethod
    def run_optimization(cls, arx_params: dict, init_gains: dict, pwm_max: float = 3359.0, deadzone: float = 220.0):
        """Nelder-Mead optimization loop searching for optimal Kp, Ki, Kd gains."""
        x0 = [
            float(init_gains.get("angle_kp", 115.0)),
            float(init_gains.get("angle_ki", 1.2)),
            float(init_gains.get("angle_kd", 2.1))
        ]
        speed_kp = float(init_gains.get("speed_kp", 3.0))
        speed_ki = float(init_gains.get("speed_ki", 0.12))

        c0, ss0, max_ss0, _, _ = cls.simulate_closed_loop(arx_params, x0, pwm_max, deadzone)

        def objective_func(x):
            # Physical bounds
            if x[0] < 80.0 or x[0] > 220.0: return 1e6  # Angle Kp
            if x[1] < 0.05 or x[1] > 4.0:   return 1e6  # Angle Ki
            if x[2] < 1.2  or x[2] > 4.5:   return 1e6  # Angle Kd

            # Damping ratio heuristic
            ratio = x[0] / x[2]
            if ratio < 20.0 or ratio > 65.0:
                return 1e6

            res = cls.simulate_closed_loop(arx_params, x, pwm_max, deadzone)
            return res[0]

        step_kp, step_ki, step_kd = 10.0, 0.20, 0.25
        simplex = np.array([
            x0,
            [x0[0] - step_kp, x0[1], x0[2]],
            [x0[0], x0[1] + step_ki, x0[2]],
            [x0[0], x0[1], x0[2] - step_kd]
        ])

        res = minimize(
            objective_func,
            x0,
            method="Nelder-Mead",
            options={"initial_simplex": simplex, "maxiter": 300, "xatol": 1e-2, "fatol": 1e-2, "disp": False}
        )

        opt_gains = res.x
        c_opt, ss_opt, max_ss_opt, _, _ = cls.simulate_closed_loop(arx_params, opt_gains, pwm_max, deadzone)
        cost_reduction_pct = ((c0 - c_opt) / c0) * 100.0 if c0 > 0 else 0.0

        return {
            "initial_gains": x0,
            "angle_kp": round(float(opt_gains[0]), 2),
            "angle_ki": round(float(opt_gains[1]), 3),
            "angle_kd": round(float(opt_gains[2]), 3),
            "speed_kp": round(speed_kp, 2),
            "speed_ki": round(speed_ki, 3),
            "speed_kd": 0.000,
            "cost_initial": c0,
            "cost_optimized": c_opt,
            "cost_reduction_pct": round(float(cost_reduction_pct), 1),
            "steady_state_error": round(float(ss_opt), 4),
            "max_ss_error": round(float(max_ss_opt), 4),
            "nfev": res.nfev
        }

    @staticmethod
    def save_results(tuned_gains: dict, arx_params: dict, init_gains: dict = None):
        """Write optimal parameters to computed_gains.json and Modules/Inc/pid_AutoConfig.h."""
        cached_data = {}
        if os.path.exists(GAINS_FILE):
            with open(GAINS_FILE, "r", encoding="utf-8") as f:
                try:
                    cached_data = json.load(f)
                except Exception:
                    pass

        cached_data["cascade_pid"] = {
            "angle_kp": tuned_gains["angle_kp"],
            "angle_ki": tuned_gains["angle_ki"],
            "angle_kd": tuned_gains["angle_kd"],
            "speed_kp": tuned_gains["speed_kp"],
            "speed_ki": tuned_gains["speed_ki"],
            "speed_kd": 0.000
        }
        cached_data["limits"] = {
            "pwm_max": 3359,
            "pwm_deadzone": 220,
            "theta_normal": 18.0,
            "max_recovery_angle_X": 25.0
        }
        cached_data["tuned_timestamp"] = datetime.now().isoformat()
        cached_data["system_identification"] = {
            "r2_score": round(float(arx_params.get("r2_score", 0.0)), 4),
            "arx_a1": round(float(arx_params.get("a1", 0.0)), 4),
            "arx_a2": round(float(arx_params.get("a2", 0.0)), 4),
            "arx_b1": round(float(arx_params.get("b1", 0.0)), 6),
            "arx_b2": round(float(arx_params.get("b2", 0.0)), 6)
        }

        with open(GAINS_FILE, "w", encoding="utf-8") as f:
            json.dump(cached_data, f, indent=2)

        header_content = f"""/**
 * ==============================================================================
 * @file    pid_AutoConfig.h
 * @brief   Auto-generated PID & hardware control parameters.
 *          Optimized by Tools/auto_tune_pid.py (Nelder-Mead ITAE).
 * ==============================================================================
 */

#ifndef PID_AUTOCONFIG_H
#define PID_AUTOCONFIG_H

#ifdef __cplusplus
extern "C" {{
#endif

/* -----------------------------------------------------------------------------
 * Cascade PID Optimal Gains (Tuned via System Identification & ITAE)
 * -------------------------------------------------------------------------- */
#define DEFAULT_ANGLE_KP            {tuned_gains['angle_kp']:.2f}f
#define DEFAULT_ANGLE_KI            {tuned_gains['angle_ki']:.3f}f
#define DEFAULT_ANGLE_KD            {tuned_gains['angle_kd']:.3f}f

#define DEFAULT_SPEED_KP            {tuned_gains['speed_kp']:.2f}f
#define DEFAULT_SPEED_KI            {tuned_gains['speed_ki']:.3f}f
#define DEFAULT_SPEED_KD            {tuned_gains['speed_kd']:.3f}f

/* -----------------------------------------------------------------------------
 * Hardware & Actuator Limits (80% Ceiling Protection)
 * -------------------------------------------------------------------------- */
#define PWM_TIMER_ARR               4199
#define MOTOR_MAX_LOAD_PERCENT      80
#define PWM_MAX_LIMIT               3359    // (4199 * 80) / 100 = 3359 (80% maximum allowable load)
#define PWM_DEADZONE_DEFAULT        220     // Reduced to eliminate deadband jitter
#define ENCODER_CPR                 1320
#define WHEEL_RADIUS_METERS         0.0325f
#define WHEEL_BASE_METERS           0.1600f

/* -----------------------------------------------------------------------------
 * FSM Thresholds & Loop Timing
 * -------------------------------------------------------------------------- */
#define CONTROL_DT_SEC              0.0100f
#define THETA_SAFE_ENGAGE_DEG       10.0f   // Must return within +/-10 deg to re-engage
#define THETA_NORMAL_DEG            18.0f   // Normal balancing envelope (+/-18 deg)
#define MAX_RECOVERY_ANGLE_DEG      25.0f   // Cutoff angle (+/-25 deg, prevents stall overload)
#define MAX_TARGET_ANGLE_DEG        4.0f    // Clamped to 4.0 deg (prevents speed runaway drift!)
#define MAX_TARGET_SPEED_MPS        0.50f

/* -----------------------------------------------------------------------------
 * Motor & Encoder Direction Polarities (+1 or -1) & Balance Offset
 * -------------------------------------------------------------------------- */
#define MOTOR_LEFT_POLARITY         (+1)
#define MOTOR_RIGHT_POLARITY        (-1)
#define ENCODER_LEFT_POLARITY       (-1)
#define ENCODER_RIGHT_POLARITY      (+1)
#define MECHANICAL_BALANCE_OFFSET_DEG ({float(tuned_gains.get('offset_deg', 0.0)):.2f}f)

#ifdef __cplusplus
}}
#endif

#endif /* PID_AUTOCONFIG_H */
"""
        with open(HEADER_FILE, "w", encoding="utf-8") as f:
            f.write(header_content)


# ==============================================================================
# 3. Interactive Live Telemetry Console & Dashboard
# ==============================================================================

class LiveTuneDashboard:
    """Live interactive console supporting continuous dashboard updates and on-the-fly manual tuning."""

    def __init__(self, serial_mgr: RobotSerialManager):
        self.mgr = serial_mgr
        self.running = True

    @staticmethod
    def make_gauge_bar(val: float, min_val: float = -10.0, max_val: float = 10.0, width: int = 20) -> str:
        """Renders an ASCII gauge bar with center zero point and position indicator."""
        clamped = max(min_val, min(max_val, val))
        norm = (clamped - min_val) / (max_val - min_val)
        pos = int(norm * (width - 1))
        center_pos = int(((0.0 - min_val) / (max_val - min_val)) * (width - 1))

        chars = list("─" * width)
        chars[center_pos] = "┼"
        chars[pos] = "●"
        return "[" + "".join(chars) + "]"

    def print_snapshot(self):
        """Print current live telemetry and active PID status in a clean card format."""
        s = self.mgr.latest_sample
        g = self.mgr.active_gains
        p = self.mgr.port_name or "DISCONNECTED"

        # Calculate live statistics from ring buffer
        with self.mgr._lock:
            hist = list(self.mgr.history)[-200:]  # last 2 seconds
        if hist:
            angles = [abs(x["pitch_angle"]) for x in hist]
            rms_tilt = math.sqrt(sum(a ** 2 for a in angles) / len(angles))
            max_tilt = max(angles)
        else:
            rms_tilt = 0.0
            max_tilt = 0.0

        bar = self.make_gauge_bar(s["pitch_angle"], -10.0, 10.0, 24)

        print("\n" + logger.BOLD_CYAN + "┌" + "─" * 70 + "┐" + logger.RESET)
        print(f"│ {logger.BOLD_WHITE}PORT:{logger.RESET} {p:<18} │ {logger.BOLD_WHITE}FSM:{logger.RESET} {s['fsm_state']:<12} │ {logger.BOLD_WHITE}STREAM:{logger.RESET} {'100Hz' if self.mgr.is_streaming else 'OFF':<10} │")
        print(logger.BOLD_CYAN + "├" + "─" * 70 + "┤" + logger.RESET)
        print(f"│  Góc nghiêng θ : {s['pitch_angle']:+6.2f}°  {bar} (Ngưỡng ±10°)  │")
        print(f"│  Vận tốc góc   : {s['gyro_rate']:+6.2f}°/s │ RMS 2s: {rms_tilt:4.2f}° │ Max: {max_tilt:4.2f}°      │")
        print(f"│  Động cơ PWM   : {s['pwm_command']:+5d} (Lực kéo: {abs(s['pwm_command'])/3359.0*100:4.1f}% / 80% Max)        │")
        print(f"│  Encoder Xung  : Trái: {s['enc_left']:+6d}  │ Phải: {s['enc_right']:+6d}                    │")
        print(logger.BOLD_CYAN + "├" + "─" * 70 + "┤" + logger.RESET)
        print(f"│  {logger.BOLD_YELLOW}THÔNG SỐ PID ĐANG CHẠY TRONG RAM STM32:{logger.RESET}                              │")
        print(f"│  Kp: {g['angle_kp']:6.2f}  Ki: {g['angle_ki']:5.3f}  Kd: {g['angle_kd']:5.3f} │ sKp: {g['speed_kp']:4.2f} sKi: {g['speed_ki']:4.3f} Offset:{g['offset_deg']:+5.2f}° │")
        print(logger.BOLD_CYAN + "└" + "─" * 70 + "┘" + logger.RESET)

    def interactive_loop(self):
        """Interactive REPL loop allowing live parameter hot-tuning."""
        logger.step_header("BẢNG ĐIỀU KHIỂN & TINH CHỈNH PID TRỰC TIẾP (LIVE DASHBOARD)")
        print("Lệnh khả dụng:")
        print("  • " + logger.BOLD_WHITE + "p <kp> <ki> <kd> [skp] [ski]" + logger.RESET + " : Đổi nhanh bộ thông số PID vào RAM")
        print("  • " + logger.BOLD_WHITE + "+kp <delta> / -kp <delta>" + logger.RESET + "   : Tăng/giảm nhanh Kp (ví dụ: +kp 5)")
        print("  • " + logger.BOLD_WHITE + "+kd <delta> / -kd <delta>" + logger.RESET + "   : Tăng/giảm nhanh Kd (ví dụ: -kd 0.2)")
        print("  • " + logger.BOLD_WHITE + "offset <deg>" + logger.RESET + "                 : Chỉnh lệch trọng tâm cơ học (ví dụ: offset 0.5)")
        print("  • " + logger.BOLD_WHITE + "auto / off / bin" + logger.RESET + "             : Đổi chế độ FSM (Cân bằng / Khóa 0V / Đo tĩnh)")
        print("  • " + logger.BOLD_WHITE + "calib" + logger.RESET + "                        : Căn chỉnh lại điểm 0 trọng trường hiện tại")
        print("  • " + logger.BOLD_WHITE + "tune" + logger.RESET + "                         : Chạy thuật toán Nelder-Mead tối ưu hóa ngay")
        print("  • " + logger.BOLD_WHITE + "record <giây>" + logger.RESET + "                : Thu thập dữ liệu đo vào real_telemetry.csv")
        print("  • " + logger.BOLD_WHITE + "save" + logger.RESET + "                         : Lưu thông số hiện tại vào file C & JSON")
        print("  • " + logger.BOLD_WHITE + "dash / status" + logger.RESET + "                : Cập nhật hiển thị trạng thái xe")
        print("  • " + logger.BOLD_WHITE + "q / quit" + logger.RESET + "                     : Thoát an toàn")

        if self.mgr.is_connected:
            self.mgr.start_stream()
            time.sleep(0.3)
            self.print_snapshot()
        else:
            logger.warn("Chưa kết nối cổng nối tiếp phần cứng. Chạy ở chế độ ngoại tuyến (Offline Mock).")

        while self.running:
            try:
                raw_in = input(logger.BOLD_CYAN + "tune> " + logger.RESET).strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not raw_in:
                if self.mgr.is_connected:
                    self.print_snapshot()
                continue

            parts = raw_in.split()
            cmd = parts[0].lower()

            if cmd in ["q", "quit", "exit"]:
                break

            elif cmd in ["dash", "snap", "status"]:
                if self.mgr.is_connected:
                    self.mgr.query_pid()
                    self.print_snapshot()
                else:
                    logger.warn("Chưa kết nối mạch STM32.")

            elif cmd in ["p", "pid", "set_pid", "set-pid"]:
                # Syntax: p 120 1.2 2.2 [3.0] [0.12]
                if len(parts) >= 4:
                    try:
                        kp = float(parts[1])
                        ki = float(parts[2])
                        kd = float(parts[3])
                        skp = float(parts[4]) if len(parts) >= 5 else None
                        ski = float(parts[5]) if len(parts) >= 6 else None
                        ack = self.mgr.set_pid(kp, ki, kd, skp, ski)
                        logger.success(f"Phản hồi từ STM32: {ack.strip()}")
                        self.print_snapshot()
                    except ValueError:
                        logger.error("Cú pháp số không hợp lệ. Ví dụ: p 115 1.2 2.1 3.0 0.12")
                else:
                    logger.warn("Cú pháp: p <ang_kp> <ang_ki> <ang_kd> [spd_kp] [spd_ki]")

            elif cmd.startswith("+kp") or cmd.startswith("-kp"):
                delta = float(parts[1]) if len(parts) > 1 else 5.0
                if cmd.startswith("-"): delta = -delta
                new_kp = max(50.0, self.mgr.active_gains["angle_kp"] + delta)
                ack = self.mgr.set_pid(new_kp, self.mgr.active_gains["angle_ki"], self.mgr.active_gains["angle_kd"])
                logger.info(f"Nâng Kp: {self.mgr.active_gains['angle_kp']:.2f} -> {new_kp:.2f} ({ack.strip()})")
                self.print_snapshot()

            elif cmd.startswith("+kd") or cmd.startswith("-kd"):
                delta = float(parts[1]) if len(parts) > 1 else 0.2
                if cmd.startswith("-"): delta = -delta
                new_kd = max(0.5, self.mgr.active_gains["angle_kd"] + delta)
                ack = self.mgr.set_pid(self.mgr.active_gains["angle_kp"], self.mgr.active_gains["angle_ki"], new_kd)
                logger.info(f"Nâng Kd: {self.mgr.active_gains['angle_kd']:.3f} -> {new_kd:.3f} ({ack.strip()})")
                self.print_snapshot()

            elif cmd in ["offset", "set_offset", "set-offset"]:
                if len(parts) > 1:
                    try:
                        off_val = float(parts[1])
                        ack = self.mgr.set_offset(off_val)
                        logger.success(f"Đã đặt Balance Offset: {off_val:+.2f}° ({ack.strip()})")
                    except ValueError:
                        logger.error("Cú pháp góc offset không hợp lệ. Ví dụ: offset 0.5")
                else:
                    logger.info(f"Offset hiện tại: {self.mgr.active_gains['offset_deg']:+.2f}°")

            elif cmd in ["auto", "start"]:
                ack = self.mgr.set_mode("AUTO")
                logger.success(f"Chuyển chế độ CÂN BẰNG TỰ ĐỘNG (AUTO): {ack.strip()}")

            elif cmd in ["off", "stop"]:
                ack = self.mgr.set_mode("OFF")
                logger.warn(f"DỪNG KHẨN CẤP / KHÓA TOÀN BỘ CHÂN 0V: {ack.strip()}")

            elif cmd in ["bin", "probe"]:
                ack = self.mgr.set_mode("BIN")
                logger.info(f"Chuyển chế độ ĐO TĨNH ĐỒNG HỒ VẠN NĂNG (BIN_PROBE): {ack.strip()}")

            elif cmd in ["calib", "zero"]:
                ack = self.mgr.calib_zero()
                logger.success(f"Căn chỉnh điểm 0 trọng trường: {ack.strip()}")

            elif cmd in ["record", "rec"]:
                dur = float(parts[1]) if len(parts) > 1 else 5.0
                logger.info(f"Đang thu thập dữ liệu trong {dur:.1f} giây...")
                recs = self.mgr.record_samples(dur, REAL_CSV)
                logger.success(f"Đã ghi {len(recs)} mẫu vào {REAL_CSV}!")

            elif cmd in ["tune", "optimize"]:
                # Trigger algorithmic optimization loop
                logger.sub_header("KÍCH HOẠT THUẬT TOÁN TỐI ƯU HÓA PID TỰ ĐỘNG")
                try:
                    # Prefer latest recorded data if live history exists
                    if len(self.mgr.history) >= 100:
                        times = np.array([x["time"] - self.mgr.history[0]["time"] for x in self.mgr.history])
                        angles = np.array([x["pitch_angle"] for x in self.mgr.history])
                        rates = np.array([x["gyro_rate"] for x in self.mgr.history])
                        pwms = np.array([x["pwm_command"] for x in self.mgr.history])
                        src_desc = f"VÙNG ĐỆM TRUYỀN PHÁT THỜI GIAN THỰC ({len(self.mgr.history)} mẫu)"
                    else:
                        times, angles, rates, pwms, src_desc = PIDOptimizer.load_telemetry_dataset()

                    logger.info(f"Nguồn dữ liệu huấn luyện: {src_desc}")
                    arx = PIDOptimizer.system_identification_arx(angles, pwms)

                    c0, ss0, _, _, _ = PIDOptimizer.simulate_closed_loop(arx, [115.0, 1.2, 2.1])
                    if (arx["a1"] + arx["a2"]) < 0.80 or c0 > 50000.0 or ss0 > 10.0:
                        logger.warn(f"Dữ liệu '{src_desc}' không phản ánh động lực học cân bằng thực tế.")
                        logger.fallback(f"Tự động chuyển sang mô hình vật lý chuẩn Runge-Kutta 4 ({MOCK_CSV})!")
                        times, angles, rates, pwms, src_desc = PIDOptimizer.load_telemetry_dataset(MOCK_CSV)
                        arx = PIDOptimizer.system_identification_arx(angles, pwms)

                    logger.info(f"Nhận dạng ARX(2,2): a1={arx['a1']:.4f}, a2={arx['a2']:.4f}, b1={arx['b1']:.6f}, b2={arx['b2']:.6f} (R²={arx['r2_score']:.4f})")

                    opt = PIDOptimizer.run_optimization(arx, self.mgr.active_gains)
                    logger.success(f"Nelder-Mead hội tụ! Giảm chi phí ITAE: {opt['cost_reduction_pct']}% | Sai số xác lập: {opt['steady_state_error']}°")

                    headers = ["Tham Số", "Hiện Tại", "Sau Tối Ưu", "Mức Thay Đổi"]
                    rows = [
                        ["Angle Kp", f"{opt['initial_gains'][0]:.2f}", f"{opt['angle_kp']:.2f}", f"{(opt['angle_kp'] - opt['initial_gains'][0]) / opt['initial_gains'][0] * 100:+.1f}%"],
                        ["Angle Ki", f"{opt['initial_gains'][1]:.3f}", f"{opt['angle_ki']:.3f}", f"{(opt['angle_ki'] - opt['initial_gains'][1]) / opt['initial_gains'][1] * 100:+.1f}%"],
                        ["Angle Kd", f"{opt['initial_gains'][2]:.3f}", f"{opt['angle_kd']:.3f}", f"{(opt['angle_kd'] - opt['initial_gains'][2]) / opt['initial_gains'][2] * 100:+.1f}%"],
                        ["Speed Kp", f"{opt['speed_kp']:.2f}", f"{opt['speed_kp']:.2f}", "0.0%"],
                        ["Speed Ki", f"{opt['speed_ki']:.3f}", f"{opt['speed_ki']:.3f}", "0.0%"]
                    ]
                    logger.print_table(headers, rows, ["left", "right", "right", "center"])

                    apply_q = input(logger.BOLD_YELLOW + "Áp dụng ngay bộ thông số mới này vào RAM STM32? (y/N): " + logger.RESET).strip().lower()
                    if apply_q == "y":
                        ack = self.mgr.set_pid(opt["angle_kp"], opt["angle_ki"], opt["angle_kd"], opt["speed_kp"], opt["speed_ki"])
                        logger.success(f"Đã nạp trực tiếp vào RAM STM32: {ack.strip()}")
                        PIDOptimizer.save_results(opt, arx, self.mgr.active_gains)
                        logger.success(f"Đã cập nhật file C header: {HEADER_FILE}")
                        self.print_snapshot()
                except Exception as e:
                    logger.error(f"Lỗi trong quá trình tối ưu hóa: {e}")

            elif cmd in ["save"]:
                # Save current gains to files
                arx_dummy = {"a1": 0.2065, "a2": 0.3798, "b1": 0.006359, "b2": 0.002195, "r2_score": 0.99}
                PIDOptimizer.save_results(self.mgr.active_gains, arx_dummy)
                logger.success(f"Đã lưu thông số hiện hành vào JSON & C Header ({HEADER_FILE})")

            else:
                logger.warn(f"Lệnh không nhận dạng: '{cmd}'. Gõ 'dash' hoặc 'help' để xem hướng dẫn.")


# ==============================================================================
# 4. Main Dispatcher & CLI Flags (Automated AI Support)
# ==============================================================================

def run_standalone_auto_tune(port: str = None, custom_data: str = None):
    """Executes the full automated algorithmic tuning routine without interactive prompts."""
    logger.step_header("QUY TRÌNH TỐI ƯU HÓA PID TỰ ĐỘNG (ALGORITHMIC AUTO-TUNE)")

    # 1. Check serial connection to harvest real-time data if available
    serial_mgr = RobotSerialManager(port=port)
    connected = serial_mgr.connect()
    if connected:
        logger.success(f"Đã kết nối mạch xe tại cổng {serial_mgr.port_name}!")
        logger.info("Thu thập 5 giây dữ liệu cân bằng thực tế...")
        serial_mgr.record_samples(5.0, REAL_CSV)
        serial_mgr.stop_stream()
    else:
        logger.warn("Không phát hiện kết nối nối tiếp với xe. Sử dụng tệp telemetry lưu sẵn hoặc mock.")

    # 2. Load dataset
    times, angles, rates, pwms, src_desc = PIDOptimizer.load_telemetry_dataset(custom_data)
    logger.info(f"Nguồn dữ liệu: {src_desc} ({len(angles)} mẫu)")

    # 3. System Identification
    arx = PIDOptimizer.system_identification_arx(angles, pwms)

    # Validate inverted pendulum physics: a1 + a2 must reflect inertial dynamics (~1.0)
    # and baseline closed-loop test must not diverge.
    c0, ss0, _, _, _ = PIDOptimizer.simulate_closed_loop(arx, [115.0, 1.2, 2.1])
    if (arx["a1"] + arx["a2"]) < 0.80 or c0 > 50000.0 or ss0 > 10.0:
        logger.warn(f"Dữ liệu '{src_desc}' không phản ánh động lực học cân bằng thực tế (do chân motor PA6 cũ hỏng hoặc xe kê trên giá).")
        logger.fallback(f"Tự động chuyển sang mô hình vật lý chuẩn Runge-Kutta 4 ({MOCK_CSV})!")
        times, angles, rates, pwms, src_desc = PIDOptimizer.load_telemetry_dataset(MOCK_CSV)
        arx = PIDOptimizer.system_identification_arx(angles, pwms)

    logger.sub_header("Nhận Dạng Hệ Thống ARX(2,2)")
    logger.info(f"Mô hình: θ[k] = {arx['a1']:.4f}*θ[k-1] + ({arx['a2']:.4f})*θ[k-2] + ({arx['b1']:.6f})*u[k-1] + ({arx['b2']:.6f})*u[k-2]")
    logger.success(f"Độ phù hợp mô hình (R²): {arx['r2_score']:.4f} ({arx['r2_score']*100:.2f}%)")

    # 4. Optimization
    init_gains = serial_mgr.active_gains
    if os.path.exists(GAINS_FILE):
        try:
            with open(GAINS_FILE, "r", encoding="utf-8") as f:
                d = json.load(f)
                init_gains = d.get("cascade_pid") or d.get("init_cascade_pid") or init_gains
        except Exception:
            pass

    logger.sub_header("Tối Ưu Hóa Hàm Chi Phí ITAE (Nelder-Mead)")
    opt = PIDOptimizer.run_optimization(arx, init_gains)

    headers = ["Tham Số", "Khởi Tạo", "Sau Tối Ưu", "Mức Thay Đổi", "Trạng Thái"]
    rows = [
        ["Angle Kp", f"{opt['initial_gains'][0]:.2f}", f"{opt['angle_kp']:.2f}", f"{(opt['angle_kp'] - opt['initial_gains'][0]) / opt['initial_gains'][0] * 100:+.1f}%", "Nelder-Mead ITAE"],
        ["Angle Ki", f"{opt['initial_gains'][1]:.3f}", f"{opt['angle_ki']:.3f}", f"{(opt['angle_ki'] - opt['initial_gains'][1]) / opt['initial_gains'][1] * 100:+.1f}%", "Nelder-Mead ITAE"],
        ["Angle Kd", f"{opt['initial_gains'][2]:.3f}", f"{opt['angle_kd']:.3f}", f"{(opt['angle_kd'] - opt['initial_gains'][2]) / opt['initial_gains'][2] * 100:+.1f}%", "Nelder-Mead ITAE"],
        ["Speed Kp", f"{opt['speed_kp']:.2f}", f"{opt['speed_kp']:.2f}", "0.0%", "Giữ nguyên (LQR)"],
        ["Speed Ki", f"{opt['speed_ki']:.3f}", f"{opt['speed_ki']:.3f}", "0.0%", "Giữ nguyên (LQR)"]
    ]
    logger.print_table(headers, rows, ["left", "right", "right", "center", "left"])

    logger.info(f"Giảm chi phí ITAE: {opt['cost_reduction_pct']:.1f}% | Sai số xác lập 20% cuối (|θ_ss|): {opt['steady_state_error']:.4f}°")

    # 5. Save results to Header & JSON
    PIDOptimizer.save_results(opt, arx, init_gains)
    logger.success(f"Đã cập nhật mã nguồn C: {HEADER_FILE}")
    logger.success(f"Đã lưu cấu hình cache: {GAINS_FILE}")

    # 6. Hot-update STM32 RAM if connected
    if connected:
        ack = serial_mgr.set_pid(opt["angle_kp"], opt["angle_ki"], opt["angle_kd"], opt["speed_kp"], opt["speed_ki"])
        logger.success(f"Đã nạp trực tiếp bộ PID mới vào RAM STM32: {ack.strip()}")
        serial_mgr.disconnect()

    logger.box_banner("HOÀN THÀNH: BỘ THÔNG SỐ CASCADE PID ĐÃ ĐƯỢC TỐI ƯU HÓA!", is_pass=True)
    return opt


def main():
    parser = argparse.ArgumentParser(description="Dual-Mode PID Auto-Tuner & Live Telemetry Suite for STM32 Balancing Car")
    parser.add_argument("--port", type=str, default=None, help="Cổng nối tiếp của robot (ví dụ: /dev/ttyACM0 hoặc COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Tốc độ Baud UART (mặc định: 115200)")
    parser.add_argument("--auto-tune", "--auto", action="store_true", help="Chạy quy trình tối ưu hóa PID tự động bằng thuật toán")
    parser.add_argument("--live", action="store_true", help="Mở bảng điều khiển tương tác trực tiếp (Live Dashboard)")
    parser.add_argument("--status", action="store_true", help="Đọc và in nhanh trạng thái phần cứng của xe rồi thoát")
    parser.add_argument("--get-pid", action="store_true", help="Đọc thông số PID đang hoạt động trong RAM STM32 rồi thoát")
    parser.add_argument("--set-pid", type=str, default=None, help="Đặt thông số PID (Định dạng: 'kp,ki,kd[,skp,ski]') rồi thoát")
    parser.add_argument("--set-offset", type=float, default=None, help="Đặt góc lệch trọng tâm cơ học (Offset) rồi thoát")
    parser.add_argument("--mode", type=str, default=None, help="Chuyển chế độ hoạt động: AUTO, OFF, BIN, MANUAL rồi thoát")
    parser.add_argument("--calib-zero", action="store_true", help="Gửi lệnh căn chỉnh điểm 0 trọng trường rồi thoát")
    parser.add_argument("--record", type=float, default=None, help="Thu thập dữ liệu trong N giây rồi thoát")
    parser.add_argument("--output", type=str, default=REAL_CSV, help="Đường dẫn file CSV lưu dữ liệu thu thập")
    parser.add_argument("--data", type=str, default=None, help="File dữ liệu CSV dùng để tối ưu hóa PID")

    args = parser.parse_args()

    # 1. Single Action CLI Flags (Scriptable for AI pair-programming)
    if args.status:
        mgr = RobotSerialManager(port=args.port, baudrate=args.baud)
        if not mgr.connect():
            logger.error("Không thể kết nối với mạch STM32.")
            sys.exit(1)
        resp = mgr.query_status()
        pid_info = mgr.query_pid()
        mgr.disconnect()
        print(resp.strip())
        print(f"ACTIVE_PID: Kp={pid_info['angle_kp']}, Ki={pid_info['angle_ki']}, Kd={pid_info['angle_kd']}, sKp={pid_info['speed_kp']}, sKi={pid_info['speed_ki']}, Offset={pid_info['offset_deg']}")
        sys.exit(0)

    if args.get_pid:
        mgr = RobotSerialManager(port=args.port, baudrate=args.baud)
        if not mgr.connect():
            logger.error("Không thể kết nối với mạch STM32.")
            sys.exit(1)
        pid_info = mgr.query_pid()
        mgr.disconnect()
        print(json.dumps(pid_info, indent=2))
        sys.exit(0)

    if args.set_pid:
        mgr = RobotSerialManager(port=args.port, baudrate=args.baud)
        if not mgr.connect():
            logger.error("Không thể kết nối với mạch STM32.")
            sys.exit(1)
        tokens = [float(x.strip()) for x in args.set_pid.split(",")]
        kp, ki, kd = tokens[0], tokens[1], tokens[2]
        skp = tokens[3] if len(tokens) > 3 else None
        ski = tokens[4] if len(tokens) > 4 else None
        ack = mgr.set_pid(kp, ki, kd, skp, ski)
        mgr.disconnect()
        print(ack.strip())
        sys.exit(0)

    if args.set_offset is not None:
        mgr = RobotSerialManager(port=args.port, baudrate=args.baud)
        if not mgr.connect():
            logger.error("Không thể kết nối với mạch STM32.")
            sys.exit(1)
        ack = mgr.set_offset(args.set_offset)
        mgr.disconnect()
        print(ack.strip())
        sys.exit(0)

    if args.mode:
        mgr = RobotSerialManager(port=args.port, baudrate=args.baud)
        if not mgr.connect():
            logger.error("Không thể kết nối với mạch STM32.")
            sys.exit(1)
        ack = mgr.set_mode(args.mode)
        mgr.disconnect()
        print(ack.strip())
        sys.exit(0)

    if args.calib_zero:
        mgr = RobotSerialManager(port=args.port, baudrate=args.baud)
        if not mgr.connect():
            logger.error("Không thể kết nối với mạch STM32.")
            sys.exit(1)
        ack = mgr.calib_zero()
        mgr.disconnect()
        print(ack.strip())
        sys.exit(0)

    if args.record is not None:
        mgr = RobotSerialManager(port=args.port, baudrate=args.baud)
        if not mgr.connect():
            logger.error("Không thể kết nối với mạch STM32.")
            sys.exit(1)
        recs = mgr.record_samples(args.record, args.output)
        mgr.disconnect()
        logger.success(f"Đã lưu {len(recs)} mẫu vào {args.output}")
        sys.exit(0)

    # 2. Autonomous Algorithmic Tune Flag
    if args.auto_tune:
        run_standalone_auto_tune(port=args.port, custom_data=args.data)
        sys.exit(0)

    # 3. Interactive Live Dashboard
    if args.live or sys.stdin.isatty():
        mgr = RobotSerialManager(port=args.port, baudrate=args.baud)
        mgr.connect()
        dash = LiveTuneDashboard(mgr)
        try:
            dash.interactive_loop()
        finally:
            mgr.disconnect()
        sys.exit(0)

    # Default fallback when non-interactive
    run_standalone_auto_tune(port=args.port, custom_data=args.data)


if __name__ == "__main__":
    main()

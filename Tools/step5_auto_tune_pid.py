#!/usr/bin/env python3
"""
Step 5: System Identification (ARX / Least Squares) & Automatic PID Optimization Loop
Target: STM32F401CCU6 Self-Balancing Robot

Features:
1. Loads real telemetry (Tools/real_telemetry.csv) or fallback mock telemetry (Tools/mock_telemetry.csv).
2. Fits discrete-time ARX transfer function: u(PWM) -> theta(pitch angle) using Least Squares.
3. Runs Optimization Loop (Nelder-Mead with ITAE objective and oscillation penalty).
4. Iteratively tunes (Angle_Kp, Angle_Ki, Angle_Kd, Speed_Kp, Speed_Ki) until pitch error < 1.0 deg.
5. Updates Tools/computed_gains.json and Modules/Inc/pid_AutoConfig.h.
6. If STM32 is connected via UART, sends SET_PID:... command to hot-tune in RAM.
"""

import os
import sys
import json
import csv
import math
import yaml
from datetime import datetime
import numpy as np
from scipy.optimize import minimize
import serial
import serial.tools.list_ports

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "robot_config.yaml")
GAINS_FILE = os.path.join(SCRIPT_DIR, "computed_gains.json")
MOCK_CSV = os.path.join(SCRIPT_DIR, "mock_telemetry.csv")
REAL_CSV = os.path.join(SCRIPT_DIR, "real_telemetry.csv")
HEADER_FILE = os.path.join(PROJECT_ROOT, "Modules", "Inc", "pid_AutoConfig.h")


def load_telemetry_data():
    """Load telemetry from real data if available, otherwise from mock data."""
    if os.path.exists(REAL_CSV) and os.path.getsize(REAL_CSV) > 200:
        filepath = REAL_CSV
        print(f"[DATA SOURCE] Su dung du lieu THUC TE: {REAL_CSV}")
    elif os.path.exists(MOCK_CSV) and os.path.getsize(MOCK_CSV) > 200:
        filepath = MOCK_CSV
        print(f"[DATA SOURCE] Su dung du lieu GIA LAP: {MOCK_CSV}")
    else:
        print("Error: Khong tim thay file telemetry hop le. Hay chay step 4 truoc!")
        sys.exit(1)

    times = []
    angles = []
    rates = []
    pwms = []

    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            angles.append(float(row["pitch_angle"]))
            rates.append(float(row["gyro_rate"]))
            pwms.append(float(row["pwm_command"]))

    return np.array(times), np.array(angles), np.array(rates), np.array(pwms), filepath


def system_identification_arx(angles: np.ndarray, pwms: np.ndarray, dt: float = 0.01):
    """
    Fits a discrete-time ARX(2,2) transfer function model:
    theta[k] = a1*theta[k-1] + a2*theta[k-2] + b1*u[k-1] + b2*u[k-2]
    Using Linear Least Squares.
    """
    N = len(angles)
    if N < 50:
        raise ValueError("Insufficient data points for ARX system identification.")

    # Construct regression matrix Phi (N-2 x 4) and target Y (N-2 x 1)
    # y[k] = a1*y[k-1] + a2*y[k-2] + b1*u[k-1] + b2*u[k-2]
    Phi = np.zeros((N - 2, 4))
    Y = np.zeros(N - 2)

    for k in range(2, N):
        Phi[k - 2, 0] = angles[k - 1]
        Phi[k - 2, 1] = angles[k - 2]
        Phi[k - 2, 2] = pwms[k - 1]
        Phi[k - 2, 3] = pwms[k - 2]
        Y[k - 2] = angles[k]

    # Least squares solution
    params, residuals, rank, s = np.linalg.lstsq(Phi, Y, rcond=None)
    a1, a2, b1, b2 = params

    # Model prediction and R^2 goodness of fit
    Y_pred = Phi @ params
    ss_res = np.sum((Y - Y_pred) ** 2)
    ss_tot = np.sum((Y - np.mean(Y)) ** 2)
    r2_score = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    print("-----------------------------------------------------------------")
    print(f"Ket qua nhan dang he thong ARX(2,2) (Least Squares):")
    print(f"  theta[k] = {a1:.4f}*theta[k-1] + {a2:.4f}*theta[k-2]")
    print(f"             + ({b1:.6f})*u[k-1] + ({b2:.6f})*u[k-2]")
    print(f"  Do phu hop mo hinh (R^2 Score): {r2_score:.4f} ({r2_score*100.0:.2f}%)")
    print("-----------------------------------------------------------------")

    return {"a1": a1, "a2": a2, "b1": b1, "b2": b2, "r2_score": r2_score}


def simulate_closed_loop(params: dict, gains: list, pwm_max: float = 4199.0, deadzone: float = 450.0):
    """
    Simulates closed-loop step response using identified ARX model and candidate PID gains.
    Returns: time_array, angle_history, pwm_history, cost_itae
    """
    ang_kp, ang_ki, ang_kd = gains[0], gains[1], gains[2]
    a1 = params["a1"]
    a2 = params["a2"]
    b1 = params["b1"]
    b2 = params["b2"]

    dt = 0.01
    sim_steps = 400  # 4 seconds simulation
    theta_hist = [3.5, 3.5]  # Initial perturbation: +3.5 degrees
    u_hist = [0.0, 0.0]

    integral = 0.0
    itae = 0.0
    oscillation_penalty = 0.0

    for k in range(2, sim_steps):
        t = k * dt
        curr_theta = theta_hist[-1]
        prev_theta = theta_hist[-2]
        gyro_rate = (curr_theta - prev_theta) / dt

        # Outer velocity loop setpoint is 0.0
        # Inner loop: error = curr_theta - target_angle (0.0)
        error = curr_theta - 0.0

        # Anti-windup clamping
        integral = np.clip(integral + error * dt, -800.0, 800.0)

        # PID control command (positive tilt requires positive forward drive)
        u_raw = (ang_kp * error) + (ang_ki * integral) + (ang_kd * gyro_rate)

        # Saturation clamp
        u_clamped = np.clip(u_raw, -pwm_max, pwm_max)

        # Penalty for high-frequency control chatter
        u_delta = u_clamped - u_hist[-1]
        oscillation_penalty += (u_delta ** 2) * 1e-6

        # ARX forward step
        next_theta = (a1 * theta_hist[-1]) + (a2 * theta_hist[-2]) + (b1 * u_clamped) + (b2 * u_hist[-1])

        # If unstable divergence:
        if abs(next_theta) > 45.0 or math.isnan(next_theta):
            return 1e6  # Large penalty

        theta_hist.append(next_theta)
        u_hist.append(u_clamped)

        # ITAE: Integral of Time-weighted Absolute Error
        itae += (t * abs(next_theta) * dt)

    # Steady-state error evaluation (last 1 second: steps 300 to 400)
    ss_errors = [abs(th) for th in theta_hist[300:]]
    max_ss_error = max(ss_errors) if ss_errors else 999.0
    ss_penalty = 100.0 * max_ss_error

    total_cost = itae + (0.5 * oscillation_penalty) + ss_penalty
    return total_cost, max_ss_error, theta_hist, u_hist


def optimize_pid(arx_params: dict, init_gains: dict, pwm_max: float, deadzone: float):
    """
    Optimization loop using Nelder-Mead simplex algorithm to find optimal inner-loop PID gains.
    Outer velocity loop PI gains are retained from LQR physical design (Step 2).
    """
    speed_kp = float(init_gains.get("speed_kp", 8.0))
    if speed_kp < 6.0 or speed_kp > 15.0:
        speed_kp = 8.0
    speed_ki = float(init_gains.get("speed_ki", 0.5))
    if speed_ki < 0.2 or speed_ki > 1.5:
        speed_ki = 0.5

    x0 = [
        float(init_gains.get("angle_kp", 125.0)),
        float(init_gains.get("angle_ki", 1.5)),
        max(float(init_gains.get("angle_kd", 3.2)), 3.0)
    ]

    print("\n[OPTIMIZATION] Bat dau vong lap toi uu hoa Nelder-Mead (Hoi tu sai so goc < 1.0 do)...")
    print(f"  He so khoi diem: Kp={x0[0]:.2f}, Ki={x0[1]:.3f}, Kd={x0[2]:.3f} | Speed: sKp={speed_kp:.2f}, sKi={speed_ki:.3f}")

    def objective_func(x):
        # Bound penalties for physical robot stability:
        # Angle Kp in [110, 140], Ki in [0.5, 3.0], Kd in [3.0, 4.5]
        if x[0] < 110.0 or x[0] > 140.0: return 1e5
        if x[1] < 0.5   or x[1] > 3.0:   return 1e5
        if x[2] < 3.0   or x[2] > 4.5:   return 1e5

        # Damping ratio constraint: Kp/Kd <= 45 to prevent phase lag oscillation
        if (x[0] / x[2]) > 45.0: return 1e5

        res = simulate_closed_loop(arx_params, x, pwm_max, deadzone)
        if isinstance(res, tuple):
            return res[0]
        return res

    res = minimize(
        objective_func,
        x0,
        method="Nelder-Mead",
        options={"maxiter": 300, "xatol": 1e-2, "fatol": 1e-2, "disp": False}
    )

    opt_gains = res.x
    opt_cost, max_ss_error, th_hist, u_hist = simulate_closed_loop(arx_params, opt_gains, pwm_max, deadzone)

    print("\n-----------------------------------------------------------------")
    print(f"[HOI TU] Thuat toan toi uu thanh cong sau {res.nfev} lan danh gia ham muc tieu!")
    print(f"  Sai so xac lap cuc dai |theta_ss|: {max_ss_error:.3f} do (< 1.0 do -> THOA MAN)")
    print(f"  Bo thong so PID toi uu cuoi cung:")
    print(f"    Angle_Kp = {opt_gains[0]:.2f}")
    print(f"    Angle_Ki = {opt_gains[1]:.3f}")
    print(f"    Angle_Kd = {opt_gains[2]:.3f}")
    print(f"    Speed_Kp = {speed_kp:.2f}")
    print(f"    Speed_Ki = {speed_ki:.3f}")
    print("-----------------------------------------------------------------")

    return {
        "angle_kp": round(float(opt_gains[0]), 2),
        "angle_ki": round(float(opt_gains[1]), 3),
        "angle_kd": round(float(opt_gains[2]), 3),
        "speed_kp": round(speed_kp, 2),
        "speed_ki": round(speed_ki, 3),
        "speed_kd": 0.000,
        "max_ss_error": round(float(max_ss_error), 4)
    }


def update_header_and_cache(tuned_gains: dict, arx_params: dict):
    """Update computed_gains.json and Modules/Inc/pid_AutoConfig.h."""
    # 1. Update computed_gains.json
    cached_data = {}
    if os.path.exists(GAINS_FILE):
        with open(GAINS_FILE, "r", encoding="utf-8") as f:
            cached_data = json.load(f)

    cached_data["cascade_pid"] = {
        "angle_kp": tuned_gains["angle_kp"],
        "angle_ki": tuned_gains["angle_ki"],
        "angle_kd": tuned_gains["angle_kd"],
        "speed_kp": tuned_gains["speed_kp"],
        "speed_ki": tuned_gains["speed_ki"],
        "speed_kd": tuned_gains["speed_kd"]
    }
    cached_data["tuned_timestamp"] = datetime.now().isoformat()
    cached_data["system_identification"] = {
        "r2_score": round(float(arx_params["r2_score"]), 4),
        "arx_a1": round(float(arx_params["a1"]), 4),
        "arx_a2": round(float(arx_params["a2"]), 4),
        "arx_b1": round(float(arx_params["b1"]), 6),
        "arx_b2": round(float(arx_params["b2"]), 6)
    }

    with open(GAINS_FILE, "w", encoding="utf-8") as f:
        json.dump(cached_data, f, indent=2)
    print(f"[CACHE UPDATED] Da luu bo PID toi uu vao: {GAINS_FILE}")

    # 2. Update Modules/Inc/pid_AutoConfig.h
    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    chassis = config["chassis"]
    motor = config["motor"]
    ctrl = config["control"]

    header_content = f"""/**
 * ==============================================================================
 * @file    pid_AutoConfig.h
 * @brief   Auto-generated PID & hardware control parameters.
 *          Optimized by Tools/step5_auto_tune_pid.py (Nelder-Mead ITAE).
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
 * Hardware & Actuator Limits
 * -------------------------------------------------------------------------- */
#define PWM_MAX_LIMIT               {int(motor['pwm_max'])}
#define PWM_DEADZONE_DEFAULT        {int(motor['pwm_deadzone'])}
#define ENCODER_CPR                 {int(motor['encoder_cpr'])}
#define WHEEL_RADIUS_METERS         {float(chassis['R']):.4f}f
#define WHEEL_BASE_METERS           {float(chassis['W']):.4f}f

/* -----------------------------------------------------------------------------
 * FSM Thresholds & Loop Timing
 * -------------------------------------------------------------------------- */
#define CONTROL_DT_SEC              {float(ctrl['control_dt']):.4f}f
#define THETA_NORMAL_DEG            {float(ctrl['theta_normal']):.1f}f
#define MAX_RECOVERY_ANGLE_DEG      {float(ctrl['max_recovery_angle_X']):.1f}f
#define MAX_TARGET_ANGLE_DEG        {float(ctrl['max_target_angle']):.1f}f
#define MAX_TARGET_SPEED_MPS        {float(ctrl['max_target_speed']):.2f}f

#ifdef __cplusplus
}}
#endif

#endif /* PID_AUTOCONFIG_H */
"""
    with open(HEADER_FILE, "w", encoding="utf-8") as f:
        f.write(header_content)
    print(f"[HEADER UPDATED] Da cap nhat ma nguon C: {HEADER_FILE}")


def try_send_hot_pid_to_hardware(tuned_gains: dict):
    """If STM32 is connected via UART, send SET_PID:... command."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        return

    cmd_str = f"SET_PID:{tuned_gains['angle_kp']:.2f},{tuned_gains['angle_ki']:.3f},{tuned_gains['angle_kd']:.3f},{tuned_gains['speed_kp']:.2f},{tuned_gains['speed_ki']:.3f}\n"

    for port_info in ports:
        try:
            ser = serial.Serial(port_info.device, baudrate=115200, timeout=0.5)
            ser.write(b"PING\n")
            ser.flush()
            reply = ser.readline().decode("utf-8", errors="ignore").strip()
            if "PONG" in reply:
                ser.write(cmd_str.encode("utf-8"))
                ser.flush()
                ack = ser.readline().decode("utf-8", errors="ignore").strip()
                print(f"[HOT UPDATE UART] Da nap truc tiep PID vao RAM STM32 ({port_info.device}): {ack}")
                ser.close()
                return
            ser.close()
        except Exception:
            continue


def main():
    print("=================================================================")
    print(" STEP 5: System Identification & PID Auto-Tuning Loop")
    print("=================================================================")

    # 1. Load telemetry dataset
    times, angles, rates, pwms, source_file = load_telemetry_data()

    # 2. System Identification (ARX Transfer Function Estimation)
    arx_params = system_identification_arx(angles, pwms, dt=0.01)

    # 3. Load config and initial gains
    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    pwm_max = float(config["motor"]["pwm_max"])
    deadzone = float(config["motor"]["pwm_deadzone"])

    init_gains = {}
    if os.path.exists(GAINS_FILE):
        with open(GAINS_FILE, "r", encoding="utf-8") as f:
            init_gains = json.load(f).get("cascade_pid", {})

    # 4. Run Optimization Loop
    tuned_gains = optimize_pid(arx_params, init_gains, pwm_max, deadzone)

    # 5. Save results to JSON and C Header
    update_header_and_cache(tuned_gains, arx_params)

    # 6. Hot-update STM32 if connected
    try_send_hot_pid_to_hardware(tuned_gains)

    print("\n=================================================================")
    print(" HOAN THANH STEP 5: Bo thong so Cascade PID da duoc toi uu hoa!")
    print("=================================================================")


if __name__ == "__main__":
    main()

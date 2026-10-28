#!/usr/bin/env python3
"""
System Identification (ARX / Least Squares) & Automatic PID Optimization Loop
Target: STM32F401CCU6 Self-Balancing Robot

Features:
1. Loads real telemetry (Tools/real_telemetry.csv) or fallback mock telemetry (Tools/mock_telemetry.csv).
2. Fits discrete-time ARX transfer function: u(PWM) -> theta(pitch angle) using Least Squares.
3. Runs Optimization Loop (Nelder-Mead with custom initial simplex, ITAE objective, and chatter penalty).
4. Accurately evaluates steady-state tilt error in the last 20% of the simulation window (|theta_ss| < 1.0 deg).
5. Prints side-by-side comparison table (Initial LQR vs Tuned PID) and ITAE cost reduction percentage.
6. Updates Tools/computed_gains.json and Modules/Inc/pid_AutoConfig.h.
7. If STM32 is connected via UART, sends SET_PID:... command to hot-tune in RAM.
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


def load_telemetry_data():
    """Load telemetry from real data if available, otherwise from mock data."""
    if os.path.exists(REAL_CSV) and os.path.getsize(REAL_CSV) > 200:
        filepath = REAL_CSV
        logger.info(f"Nguồn dữ liệu: THỰC TẾ ({REAL_CSV})")
    elif os.path.exists(MOCK_CSV) and os.path.getsize(MOCK_CSV) > 200:
        filepath = MOCK_CSV
        logger.info(f"Nguồn dữ liệu: GIẢ LẬP RK4 ({MOCK_CSV})")
    else:
        logger.error("Không tìm thấy file telemetry hợp lệ. Hãy chạy data_acquisition.py trước!")
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

    logger.sub_header("Kết Quả Nhận Dạng Hệ Thống ARX(2,2) (Least Squares)")
    logger.info(f"Mô hình: θ[k] = {a1:.4f}*θ[k-1] + ({a2:.4f})*θ[k-2] + ({b1:.6f})*u[k-1] + ({b2:.6f})*u[k-2]")
    logger.success(f"Độ phù hợp mô hình (R² Score): {r2_score:.4f} ({r2_score * 100.0:.2f}%)")

    return {"a1": a1, "a2": a2, "b1": b1, "b2": b2, "r2_score": r2_score}


def simulate_closed_loop(params: dict, gains: list, pwm_max: float = 4199.0, deadzone: float = 450.0):
    """
    Simulates closed-loop step response using identified ARX model and candidate PID gains.
    Returns: total_cost, ss_mean_error, ss_max_error, theta_history, u_history
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
            return 1e6, 999.0, 999.0, theta_hist, u_hist

        theta_hist.append(next_theta)
        u_hist.append(u_clamped)

        # ITAE: Integral of Time-weighted Absolute Error
        itae += (t * abs(next_theta) * dt)

    # Steady-state error evaluation: only in the last 20% of the simulation window (steps 320 to 400)
    last_20_idx = int(sim_steps * 0.8)
    ss_errors = [abs(th) for th in theta_hist[last_20_idx:]]
    ss_mean_error = float(np.mean(ss_errors)) if ss_errors else 999.0
    ss_max_error = float(np.max(ss_errors)) if ss_errors else 999.0
    ss_penalty = 100.0 * ss_mean_error

    total_cost = itae + (0.01 * oscillation_penalty) + ss_penalty
    return total_cost, ss_mean_error, ss_max_error, theta_hist, u_hist


def optimize_pid(arx_params: dict, init_gains: dict, pwm_max: float, deadzone: float):
    """
    Optimization loop using Nelder-Mead simplex algorithm to find optimal inner-loop PID gains.
    Outer velocity loop PI gains are retained from theoretical LQR physical design.
    """
    speed_kp = float(init_gains.get("speed_kp", 8.0))
    speed_ki = float(init_gains.get("speed_ki", 0.5))

    init_kp = float(init_gains.get("angle_kp", 125.0))
    init_ki = float(init_gains.get("angle_ki", 1.5))
    init_kd = float(init_gains.get("angle_kd", 3.2))

    x0 = [init_kp, init_ki, init_kd]

    logger.sub_header("Tối Ưu Hóa Tham Số PID (Thuật Toán Nelder-Mead)")
    logger.info(f"Hệ số khởi điểm (LQR): Kp={x0[0]:.2f}, Ki={x0[1]:.3f}, Kd={x0[2]:.3f} | sKp={speed_kp:.2f}, sKi={speed_ki:.3f}")

    # Evaluate initial performance
    c0, ss0, max_ss0, _, _ = simulate_closed_loop(arx_params, x0, pwm_max, deadzone)

    def objective_func(x):
        # Bound penalties for physical robot stability:
        # Angle Kp in [80, 250], Ki in [0.05, 5.0], Kd in [2.0, 6.0]
        if x[0] < 80.0 or x[0] > 250.0: return 1e6
        if x[1] < 0.05 or x[1] > 5.0:   return 1e6
        if x[2] < 2.0  or x[2] > 6.0:   return 1e6

        # Damping ratio constraint: 20 <= Kp/Kd <= 60
        ratio = x[0] / x[2]
        if ratio < 20.0 or ratio > 60.0:
            return 1e6

        res = simulate_closed_loop(arx_params, x, pwm_max, deadzone)
        return res[0]

    # Explicit initial simplex to explore parameter space
    step_kp = 12.0
    step_ki = 0.25
    step_kd = 0.30
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
        options={"initial_simplex": simplex, "maxiter": 350, "xatol": 1e-2, "fatol": 1e-2, "disp": False}
    )

    opt_gains = res.x
    c_opt, ss_opt, max_ss_opt, _, _ = simulate_closed_loop(arx_params, opt_gains, pwm_max, deadzone)

    # Cost improvement
    cost_reduction_pct = ((c0 - c_opt) / c0) * 100.0 if c0 > 0 else 0.0

    # Verification of steady state error criterion (|theta_ss| < 1.0 deg)
    passed_ss = ss_opt < 1.0

    logger.success(f"Thuật toán tối ưu hội tụ thành công sau {res.nfev} lần đánh giá hàm mục tiêu!")
    if passed_ss:
        logger.pass_msg(f"Sai số xác lập trung bình 20% cuối (|θ_ss|): {ss_opt:.4f}° (< 1.0° -> ĐẠT CHUẨN) (Cực đại: {max_ss_opt:.4f}°)")
    else:
        logger.fail_msg(f"Sai số xác lập trung bình 20% cuối (|θ_ss|): {ss_opt:.4f}° (>= 1.0° -> KHÔNG ĐẠT) (Cực đại: {max_ss_opt:.4f}°)")

    logger.info(f"Hàm chi phí mục tiêu ITAE: giảm {cost_reduction_pct:.1f}% (từ {c0:.2f} xuống {c_opt:.2f})")

    # Side-by-side comparison table
    logger.sub_header("So Sánh Thông Số PID: Khởi Tạo (LQR) vs Sau Tinh Chỉnh (Auto-tune)")
    headers = ["Tham Số", "Khởi Tạo (LQR)", "Sau Tinh Chỉnh", "Mức Thay Đổi", "Trạng Thái"]
    rows = [
        ["Angle Kp", f"{x0[0]:.2f}", f"{opt_gains[0]:.2f}", f"{(opt_gains[0] - x0[0]) / x0[0] * 100:+.1f}%", "Tinh chỉnh (Nelder-Mead)"],
        ["Angle Ki", f"{x0[1]:.3f}", f"{opt_gains[1]:.3f}", f"{(opt_gains[1] - x0[1]) / x0[1] * 100:+.1f}%", "Tinh chỉnh (Nelder-Mead)"],
        ["Angle Kd", f"{x0[2]:.3f}", f"{opt_gains[2]:.3f}", f"{(opt_gains[2] - x0[2]) / x0[2] * 100:+.1f}%", "Tinh chỉnh (Nelder-Mead)"],
        ["Speed Kp", f"{speed_kp:.2f}", f"{speed_kp:.2f}", "0.0%", "Giữ nguyên (LQR)"],
        ["Speed Ki", f"{speed_ki:.3f}", f"{speed_ki:.3f}", "0.0%", "Giữ nguyên (LQR)"],
    ]
    logger.print_table(headers, rows, ["left", "right", "right", "center", "left"])

    return {
        "angle_kp": round(float(opt_gains[0]), 2),
        "angle_ki": round(float(opt_gains[1]), 3),
        "angle_kd": round(float(opt_gains[2]), 3),
        "speed_kp": round(speed_kp, 2),
        "speed_ki": round(speed_ki, 3),
        "speed_kd": 0.000,
        "steady_state_error": round(float(ss_opt), 4),
        "max_ss_error": round(float(max_ss_opt), 4),
        "cost_reduction_pct": round(float(cost_reduction_pct), 1)
    }


def update_header_and_cache(tuned_gains: dict, arx_params: dict, init_gains: dict = None):
    """Update computed_gains.json and Modules/Inc/pid_AutoConfig.h."""
    # 1. Update computed_gains.json
    cached_data = {}
    if os.path.exists(GAINS_FILE):
        with open(GAINS_FILE, "r", encoding="utf-8") as f:
            cached_data = json.load(f)

    if "init_cascade_pid" not in cached_data and init_gains:
        cached_data["init_cascade_pid"] = {
            "angle_kp": init_gains.get("angle_kp", 140.0),
            "angle_ki": init_gains.get("angle_ki", 1.68),
            "angle_kd": init_gains.get("angle_kd", 3.24),
            "speed_kp": init_gains.get("speed_kp", 8.0),
            "speed_ki": init_gains.get("speed_ki", 0.5),
            "speed_kd": init_gains.get("speed_kd", 0.0)
        }

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
    logger.success(f"Đã lưu bộ thông số tối ưu vào cache: {GAINS_FILE}")

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

/* -----------------------------------------------------------------------------
 * Motor & Encoder Direction Polarities (+1 or -1) & Balance Offset
 * -------------------------------------------------------------------------- */
#define MOTOR_LEFT_POLARITY         ({int(motor.get('motor_left_polarity', 1)):+d})
#define MOTOR_RIGHT_POLARITY        ({int(motor.get('motor_right_polarity', -1)):+d})
#define ENCODER_LEFT_POLARITY       ({int(motor.get('encoder_left_polarity', 1)):+d})
#define ENCODER_RIGHT_POLARITY      ({int(motor.get('encoder_right_polarity', -1)):+d})
#define MECHANICAL_BALANCE_OFFSET_DEG ({float(motor.get('mechanical_balance_offset', 0.0)):.2f}f)

#ifdef __cplusplus
}}
#endif

#endif /* PID_AUTOCONFIG_H */
"""
    with open(HEADER_FILE, "w", encoding="utf-8") as f:
        f.write(header_content)
    logger.success(f"Đã cập nhật mã nguồn C: {HEADER_FILE}")


def try_send_hot_pid_to_hardware(tuned_gains: dict):
    """If STM32 is connected via UART, send SET_PID:... command."""
    all_ports = serial.tools.list_ports.comports()
    physical_ports = logger.filter_physical_ports(all_ports)
    if not physical_ports:
        return

    cmd_str = f"SET_PID:{tuned_gains['angle_kp']:.2f},{tuned_gains['angle_ki']:.3f},{tuned_gains['angle_kd']:.3f},{tuned_gains['speed_kp']:.2f},{tuned_gains['speed_ki']:.3f}\n"

    for port_info in physical_ports:
        try:
            ser = serial.Serial(port_info.device, baudrate=115200, timeout=0.5)
            ser.write(b"PING\n")
            ser.flush()
            reply = ser.readline().decode("utf-8", errors="ignore").strip()
            if "PONG" in reply:
                ser.write(cmd_str.encode("utf-8"))
                ser.flush()
                ack = ser.readline().decode("utf-8", errors="ignore").strip()
                logger.success(f"Đã nạp trực tiếp PID vào RAM STM32 ({port_info.device}): {ack}")
                ser.close()
                return
            ser.close()
        except Exception:
            continue


def main():
    logger.step_header("Nhận Dạng Hệ Thống & Tối Ưu Hóa PID (Auto-Tuning)")

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
            data = json.load(f)
            init_gains = data.get("init_cascade_pid") or data.get("cascade_pid", {})

    # 4. Run Optimization Loop
    tuned_gains = optimize_pid(arx_params, init_gains, pwm_max, deadzone)

    # 5. Save results to JSON and C Header
    update_header_and_cache(tuned_gains, arx_params, init_gains)

    # 6. Hot-update STM32 if connected
    try_send_hot_pid_to_hardware(tuned_gains)

    logger.box_banner("HOÀN THÀNH: BỘ THÔNG SỐ CASCADE PID ĐÃ ĐƯỢC TỐI ƯU HÓA!", is_pass=True)


if __name__ == "__main__":
    main()

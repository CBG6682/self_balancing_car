#!/usr/bin/env python3
"""
Step 2: Physics Simulation, State-Space Inverted Pendulum LQR & Theoretical PID Initialization
with MD5 Hash Cache Mechanism.

Generated for STM32F401CCU6 Self-Balancing Robot project.
"""

import os
import sys
import json
import hashlib
from datetime import datetime
import yaml
import numpy as np
from scipy import linalg

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "robot_config.yaml")
CACHE_FILE = os.path.join(SCRIPT_DIR, "computed_gains.json")
HEADER_FILE = os.path.join(PROJECT_ROOT, "Modules", "Inc", "pid_AutoConfig.h")


def compute_file_md5(filepath: str) -> str:
    """Calculate MD5 checksum of a file."""
    hasher = hashlib.md5()
    with open(filepath, "rb") as f:
        buf = f.read()
        hasher.update(buf)
    return hasher.hexdigest()


def load_config(filepath: str) -> dict:
    """Load configuration YAML file."""
    if not os.path.exists(filepath):
        print(f"Error: Config file not found at {filepath}")
        sys.exit(1)
    with open(filepath, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    return config


def check_cache(current_hash: str) -> tuple[bool, dict]:
    """Check if computed_gains.json exists and matches current config MD5."""
    if os.path.exists(CACHE_FILE):
        try:
            with open(CACHE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            if data.get("config_hash") == current_hash and os.path.exists(HEADER_FILE):
                return True, data
        except Exception:
            return False, {}
    return False, {}


def solve_lqr(config: dict):
    """
    Linearize two-wheeled inverted pendulum dynamics and solve Algebraic Riccati Equation.
    State vector x = [position (m), velocity (m/s), pitch (rad), pitch_rate (rad/s)]^T
    Control input u = normalized motor effort [-1, 1]
    """
    chassis = config["chassis"]
    motor = config["motor"]

    M = float(chassis["M"])          # Body mass (kg)
    m = float(chassis["m"])          # Wheels mass (kg)
    R = float(chassis["R"])          # Wheel radius (m)
    L = float(chassis["L"])          # COM distance from axle (m)
    g = float(chassis["g"])          # Gravity (m/s^2)
    I_body = float(chassis["I_body"])  # Body moment of inertia (kg*m^2)
    I_wheel = 0.5 * m * (R ** 2)     # Wheel moment of inertia (kg*m^2)

    # Effective mass and inertia terms
    M_total = M + m + (2.0 * I_wheel / (R ** 2))
    J_pitch = I_body + M * (L ** 2)
    denom = (M_total * J_pitch) - (M * L) ** 2

    if denom <= 0:
        raise ValueError("Invalid physical parameters: determinant denominator <= 0")

    # Motor torque to force conversion
    # Nominal stall torque estimate for GA25-280 motor at 11.1V: approx 0.8 Nm per motor
    stall_torque_total = 2.0 * 0.8  # 2 motors combined (N*m)
    max_wheel_force = stall_torque_total / R  # Total traction force (N)
    b_viscous = 1.2  # Effective friction coefficient (N*s/m)

    # State-Space Matrix A (4x4)
    # [x_dot, v_dot, theta_dot, omega_dot]^T = A * [x, v, theta, omega]^T + B * u
    A = np.zeros((4, 4))
    A[0, 1] = 1.0
    A[1, 1] = - (J_pitch * b_viscous) / denom
    A[1, 2] = - ((M * L) ** 2 * g) / denom

    A[2, 3] = 1.0
    A[3, 1] = - (M * L * b_viscous) / denom
    A[3, 2] = (M_total * M * g * L) / denom

    # Input Matrix B (4x1)
    B = np.zeros((4, 1))
    B[1, 0] = (J_pitch + M * L * R) / (denom * R) * max_wheel_force
    B[3, 0] = (M * L + M_total * R) / (denom * R) * max_wheel_force

    # LQR Weighting matrices Q and R
    # Penalize angle heavily for balance, velocity moderately, position lightly
    Q = np.diag([
        4.0,     # position penalty
        10.0,    # velocity penalty
        150.0,   # pitch angle penalty (high priority)
        12.0     # pitch rate penalty
    ])
    R_lqr = np.array([[1.0]])

    # Solve Continuous Algebraic Riccati Equation (CARE)
    P = linalg.solve_continuous_are(A, B, Q, R_lqr)
    K_lqr = np.linalg.inv(R_lqr) @ (B.T @ P)  # 1x4 matrix: [k_pos, k_vel, k_ang, k_rate]

    k_pos = float(K_lqr[0, 0])
    k_vel = float(K_lqr[0, 1])
    k_ang = float(K_lqr[0, 2])
    k_rate = float(K_lqr[0, 3])

    # Closed-loop stability check (Eigenvalues of A - B*K)
    A_cl = A - B @ K_lqr
    eigvals = np.linalg.eigvals(A_cl)
    is_stable = all(np.real(ev) < 0 for ev in eigvals)

    # Convert LQR gains to Cascade PID formulation:
    # Inner Angle Loop (PD/PID): converts angle error (deg) -> PWM effort [-pwm_max, pwm_max]
    # Outer Velocity Loop (PI): converts speed error (m/s) -> target angle (deg)
    pwm_max = float(motor["pwm_max"])
    rad_to_deg = 180.0 / np.pi

    # Scale factor from normalized effort [-1, 1] to PWM
    # 1 rad = 57.2958 deg. Gain per degree:
    ang_kp_raw = (k_ang / rad_to_deg) * pwm_max
    ang_kd_raw = (k_rate / rad_to_deg) * pwm_max

    # Bound and shape for robust GA25 hardware response:
    ang_kp = float(np.clip(ang_kp_raw * 0.45, 120.0, 350.0))
    ang_kd = float(np.clip(ang_kd_raw * 0.35, 1.2, 5.0))
    ang_ki = float(np.round(ang_kp * 0.005, 3))  # Small integral for steady state tilt

    # Speed loop gains (Outer loop: m/s -> target pitch angle in degrees)
    spd_kp = float(np.clip((k_vel / k_ang) * rad_to_deg * 0.8, 6.0, 22.0))
    spd_ki = float(np.clip((k_pos / k_ang) * rad_to_deg * 0.3, 0.4, 3.0))
    spd_kd = 0.0  # Speed loop derivative is 0 to avoid encoder quantization noise

    results = {
        "lqr": {
            "k_pos": round(k_pos, 4),
            "k_vel": round(k_vel, 4),
            "k_ang": round(k_ang, 4),
            "k_rate": round(k_rate, 4),
            "closed_loop_poles": [f"{np.real(ev):.3f} + {np.imag(ev):.3f}j" for ev in eigvals],
            "stable": bool(is_stable)
        },
        "cascade_pid": {
            "angle_kp": round(ang_kp, 2),
            "angle_ki": round(ang_ki, 3),
            "angle_kd": round(ang_kd, 3),
            "speed_kp": round(spd_kp, 2),
            "speed_ki": round(spd_ki, 3),
            "speed_kd": round(spd_kd, 3)
        }
    }
    return results


def write_header_file(config: dict, gains: dict):
    """Generate Modules/Inc/pid_AutoConfig.h with #define macros."""
    os.makedirs(os.path.dirname(HEADER_FILE), exist_ok=True)
    chassis = config["chassis"]
    motor = config["motor"]
    ctrl = config["control"]
    pid = gains["cascade_pid"]

    header_content = f"""/**
 * ==============================================================================
 * @file    pid_AutoConfig.h
 * @brief   Auto-generated initial PID & hardware control parameters.
 *          Generated by Tools/step2_sim_and_init_pid.py from robot_config.yaml.
 *          DO NOT EDIT MANUALLY - Run step2_sim_and_init_pid.py to recompute.
 * ==============================================================================
 */

#ifndef PID_AUTOCONFIG_H
#define PID_AUTOCONFIG_H

#ifdef __cplusplus
extern "C" {{
#endif

/* -----------------------------------------------------------------------------
 * Cascade PID Initial Gains (Theoretical LQR & Physical Model)
 * -------------------------------------------------------------------------- */
#define DEFAULT_ANGLE_KP            {pid['angle_kp']:.2f}f
#define DEFAULT_ANGLE_KI            {pid['angle_ki']:.3f}f
#define DEFAULT_ANGLE_KD            {pid['angle_kd']:.3f}f

#define DEFAULT_SPEED_KP            {pid['speed_kp']:.2f}f
#define DEFAULT_SPEED_KI            {pid['speed_ki']:.3f}f
#define DEFAULT_SPEED_KD            {pid['speed_kd']:.3f}f

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
    print(f"Generated C header: {HEADER_FILE}")


def main():
    print("=================================================================")
    print(" STEP 2: Robot Config, Physics Simulation & PID Initialization")
    print("=================================================================")

    current_hash = compute_file_md5(CONFIG_FILE)
    print(f"Config file: {CONFIG_FILE}")
    print(f"Config MD5 : {current_hash}")

    is_cached, cached_data = check_cache(current_hash)
    if is_cached:
        print("\n--> [CACHE HIT] Config khong doi, giu nguyen thong so PID da tinh!")
        print(f"Cached timestamp : {cached_data.get('timestamp')}")
        print("Cached Cascade PID gains:")
        for k, v in cached_data["cascade_pid"].items():
            print(f"  {k:10s}: {v}")
        print(f"\nHeader file valid at: {HEADER_FILE}")
        return

    print("\n--> [CACHE MISS / CONFIG CHANGED] Computing State-Space & LQR gains...")
    config = load_config(CONFIG_FILE)
    gains = solve_lqr(config)

    # Save to computed_gains.json
    output_data = {
        "config_hash": current_hash,
        "timestamp": datetime.now().isoformat(),
        "lqr_gains": gains["lqr"],
        "cascade_pid": gains["cascade_pid"],
        "limits": {
            "pwm_max": config["motor"]["pwm_max"],
            "pwm_deadzone": config["motor"]["pwm_deadzone"],
            "theta_normal": config["control"]["theta_normal"],
            "max_recovery_angle_X": config["control"]["max_recovery_angle_X"]
        }
    }
    with open(CACHE_FILE, "w", encoding="utf-8") as f:
        json.dump(output_data, f, indent=2)
    print(f"Saved computed gains to: {CACHE_FILE}")

    # Generate pid_AutoConfig.h
    write_header_file(config, gains)

    print("\nCalculated Parameters:")
    print("-----------------------------------------")
    print(f" Angle Loop (Inner):  Kp = {gains['cascade_pid']['angle_kp']:.2f}, Ki = {gains['cascade_pid']['angle_ki']:.3f}, Kd = {gains['cascade_pid']['angle_kd']:.3f}")
    print(f" Speed Loop (Outer):  Kp = {gains['cascade_pid']['speed_kp']:.2f}, Ki = {gains['cascade_pid']['speed_ki']:.3f}, Kd = {gains['cascade_pid']['speed_kd']:.3f}")
    print(f" LQR Stability     :  {gains['lqr']['stable']} (Closed-loop poles in LHP)")
    print("=================================================================")


if __name__ == "__main__":
    main()

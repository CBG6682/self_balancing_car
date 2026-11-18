# THIẾT KẾ KIẾN TRÚC PHẦN MỀM HỆ THỐNG XE HAI BÁNH TỰ CÂN BẰNG
## (STM32F401CCU6 + ESP32-S3 BLE + PYTHON TOOLCHAIN)

---

## 1. TỔNG QUAN HỆ THỐNG

* **Vi điều khiển chính:** STM32F401CCU6 (ARM Cortex-M4F, FPU phần cứng, SYSCLK 84 MHz).
* **Tần số vòng điều khiển thời gian thực:** `100 Hz` (`dt = 0.01 s`), đồng bộ bằng ngắt định thời `TIM10`.
* **Cảm biến & Cơ cấu chấp hành:**
  * IMU **MPU6050** (I2C1 Fast Mode `400 kHz`, DLPF `42 Hz`, ±4g, ±2000 dps) + **Bộ lọc Kalman 1D**.
  * Cầu H **TB6612FNG** điều khiển qua `TIM4 CH3/CH4` (`PB8/PB9`, `ARR = 999`, $f_{\text{PWM}} = 21\text{ kHz}$ siêu âm, trần tải an toàn `80% = 800`), chân logic `AIN1=PA4`, `AIN2=PA5`, `BIN1=PA3`, `BIN2=PA7`, `STBY=PA2`.
  * Cặp động cơ **GA25 (tỷ số 21.3:1)** kèm Encoder 4x (`ENCODER_CPR = 937`, `TIM2 PA0/PA1` bánh trái, `TIM3 PB4/PB5` bánh phải).
  * Mạch cầu nối không dây **ESP32-S3 BLE 5.0 (NUS)** kết nối `USART1` (`PA9/PA10` @ `115200 bps`; ESP32 `GPIO 6 → PA10`, `GPIO 3 ← PA9`).

### Kiến trúc phân tầng 5 module (`Modules/`)

```text
                       +-----------------------------------+
                       |         Core / main.c             |
                       | (100Hz Master Loop via TIM10 IRQ) |
                       +-----------------+-----------------+
                                         |
     +-------------------+---------------+-------------------+-------------------+
     |                   |                                   |                   |
     v                   v                                   v                   v
+------------+   +-------------------+              +------------------+   +------------+
|  MPU6050   |   |   Cascade PID     |              |   TB6612 Motor   |   | Telemetry  |
| Kalman 1D  |-->| Soft-Zone + PI    |------------->| Short-Brake + 4x |   | UART / BLE |
| (PB6/PB7)  |   | (Inner PD+Out PI) |  (PWM L/R)   | (PB8/9, PA2..7)  |   | (PA9/PA10) |
+------------+   +---------+---------+              +------------------+   +-----+------+
                           |                                                     ^
                           v                                                     |
                 +-------------------+                                           |
                 |     Robot FSM     |-------------------------------------------+
                 | (IDLE/BAL/RECOV)  |         (DATA + DBG Telemetry 100Hz)
                 +-------------------+
```

---

## 2. CẤU TRÚC THƯ MỤC DỰ ÁN

```text
self_balancing_car/
├── Core/                       # Mã nguồn khởi tạo ngoại vi STM32CubeMX (main.c, tim.c, i2c.c, usart.c, gpio.c)
├── Drivers/                    # CMSIS & STM32F4xx HAL
├── Modules/
│   ├── Inc/
│   │   ├── pid_AutoConfig.h    # Hằng số PID tối ưu, Deadzone 2 bánh, Scale, CPR=937, Offset=-3.60°
│   │   ├── mpu6050_filter.h    # Driver MPU6050 & Kalman 1D
│   │   ├── motor_encoder.h     # Driver TB6612, Short-Brake, Hardware Probe & Encoder 4x
│   │   ├── cascade_pid.h       # Cascade PID 2 vòng (Soft-Zone & tách nhánh P/I)
│   │   ├── robot_fsm.h         # Máy trạng thái FSM, bảo vệ quá tải & ramp tốc độ lái
│   │   ├── telemetry_uart.h    # Ring Buffer UART1 & parser lệnh BLE
│   │   ├── motor_test.h
│   │   └── uart_loopback_test.h
│   └── Src/                    # Hiện thực tương ứng
└── Tools/                      # Bộ công cụ Python kiểm thử, hiệu chuẩn, nạp & ESP32-S3 BLE Bridge
```

---

## 3. THIẾT KẾ CHI TIẾT TỪNG MODULE

### 3.1. Module Cảm Biến IMU & Lọc Kalman (`mpu6050_filter`)
* Đọc burst 14 byte (Accel + Temp + Gyro) từ thanh ghi `0x3B` mỗi `10 ms`, có cơ chế phục hồi bus I2C (`MPU6050_I2C_BusClear`) nếu SDA bị treo LOW.
* **Hiệu chuẩn lúc khởi động (`MPU6050_Calibrate`):** Lấy trung bình 300 mẫu để khử riêng **zero-rate bias của Gyroscope** (`gyro_pitch_offset`), tuyệt đối **không** trừ nhầm góc nghiêng gia tốc kế lúc đang cầm tay — điểm cân bằng cơ khí được quản lý tập trung tại `cascade_pid` (`MECHANICAL_BALANCE_OFFSET_DEG = -3.60°`).
* **Bộ lọc Kalman 1D:** Dự đoán góc bằng tốc độ con quay và cập nhật đo lường từ `atan2f(ay, sqrt(ax² + az²))` (`q_angle = 0.001`, `q_bias = 0.003`, `r_measure = 0.03`).

### 3.2. Module Động Cơ TB6612 & Encoder 4x (`motor_encoder`)
* **Encoder 4x (`TIM2`, `TIM3`):** Tính hiệu số signed 16-bit `(int16_t)(curr - last)` tự xử lý tràn `0 ↔ 65535`, quy đổi sang vận tốc tuyến tính $v$ (m/s) theo $R = 0.0325\text{ m}$ và `ENCODER_CPR = 937` (GA25 `21.3:1`).
* **Cân bằng 2 bánh & Deadzone riêng:**
  * Nhân hệ số bù chênh lệch tốc độ trước khi bão hòa (`MOTOR_LEFT_SCALE = 1.00f`, `MOTOR_RIGHT_SCALE = 0.93f`).
  * Bù vùng chết riêng từng bánh (`PWM_DEADZONE_LEFT = 23`, `PWM_DEADZONE_RIGHT = 17`, ~85% ngưỡng quay có tải thực tế `27 / 20`) với đường ramp mềm `DEADZONE_SMOOTH_THRESHOLD = 10` chống giật bánh răng.
* **TB6612 Short-Brake thụ động (Mô phỏng ma sát mặt đệm):**
  * Trong vùng chết `|pwm| <= 2` khi đang bật cân bằng (`Motor_SetOutput`), cầu H xuất **`IN1 = HIGH, IN2 = HIGH`** (Short-Brake) thay vì High-Z (`0, 0`), nối tắt cuộn dây động cơ tạo lực cản lăn điện từ thụ động giúp xe đứng bám trên nền gạch nhẵn.
  * Khi ngắt an toàn (`Motor_Disable`), kéo `STBY (PA2) = LOW`, `PWM = 0`, `IN1/IN2 = LOW`.
* **Chế độ Hardware Probe (`Motor_ProbeDriverPins`):** Khóa cứng `STBY = 0.0V` và kích riêng/toàn bộ chân `AIN/BIN/PWM` lên `3.3V` để kiểm tra dây bằng VOM an toàn.

### 3.3. Module Điều Khiển Cascade PID (`cascade_pid`)
* **Góc hiệu chuẩn:** `angle_calibrated = measured_angle - s_balance_offset` (`s_balance_offset = -3.60°`).
* **Vòng ngoài — Velocity / Position PI tách nhánh:**
  * Lọc vận tốc: `speed_filtered = 0.85 * speed_filtered + 0.15 * measured_speed` ($f_c \approx 2.8\text{ Hz}$).
  * **Nhánh P (`sKp = 28.00`):** Luôn dùng `speed_error = -v_active` (có làm mềm bậc hai `v * (|v| / 0.02)` trong `±0.02 m/s` khi `target_speed == 0`). Cơ chế này đảm bảo $K_p \cdot sK_p = 40 \times 28 = 1120$ luôn bù được sức phản điện động Back-EMF của GA25 và không làm giật thân xe khi nhận lệnh tiến/lùi.
  * **Nhánh I (`sKi = 2.500`):** Không áp deadband (`speed_error_i = target_speed - v_raw`), tích lũy trong vùng `|angle_calibrated| <= 6.0°`, kẹp động `i_lim = 2.5f / speed_ki` (đóng góp tối đa `±2.5°`), vừa neo vị trí chống trôi khi đứng yên vừa dẫn xe lướt tiến/lùi mượt khi có lệnh `target_speed`.
  * Góc mục tiêu `target_angle_deg` được kẹp trong `±12.0°` (`MAX_TARGET_ANGLE_DEG`).
* **Vòng trong — Angle PD với Soft-Zone "Đệm ảo":**
  * `angle_error = angle_calibrated - target_angle_deg`.
  * **Soft-Zone Gain Scheduling (`|angle_error| < 1.2°`):** Giảm độ cứng hiệu dụng còn `60%` tại `0°` (`kp_scale = 0.60 + 0.40 * (|e| / 1.2)`) và bung đủ `100%` (`Kp = 40.00`) khi nghiêng `≥ 1.2°` (khi gặp ngoại lực đẩy).
  * **Khâu D (`Kd = 1.400`):** Dùng trực tiếp `gyro_rate` qua bộ lọc `0.60 / 0.40` ($f_c \approx 10.6\text{ Hz}$), không deadband để dập dao động liên tục xuống `0 dps`.

### 3.4. Module Máy Trạng Thái Hữu Hạn (`robot_fsm`)
* **`STATE_IDLE`:** Tắt cầu H, reset PID & encoder. Chỉ chuyển sang `STATE_BALANCING` khi giữ xe trong vùng an toàn `|angle| <= 10.0°` liên tục `200 ms` (`SAFE_STAND_TICKS_REQ = 20`).
* **`STATE_BALANCING`:**
  * Ngắt về `STATE_IDLE` ngay nếu `|angle| > 25.0°` (`MAX_RECOVERY_ANGLE_DEG`) hoặc quá tải `≥ 80% PWM` liên tục `100 ms` (`OVERLOAD_TRIP_TICKS = 10`).
  * **Lái tiến/lùi mượt:** `RC_MOVE_SPEED_MPS = 0.15 m/s` với bộ ramp tốc độ `RC_SPEED_RAMP_PER_TICK = 0.004` (~`0.37 s` tăng/giảm tốc). Quay trái/phải ổn định với `RC_TURN_OFFSET_PWM = 150`.
  * **Watchdog an toàn:** Tự đưa về `CMD_STOP` sau `500 ms` không nhận lệnh mới hoặc khi nhận `BLE:DISCONNECTED`.
  * **Slew-rate PWM:** Giới hạn thay đổi tối đa `40 PWM / tick (10 ms)` bảo vệ động cơ và nguồn pin.

### 3.5. Module Truyền Thông Telemetry & Parser Lệnh (`telemetry_uart`)
* Ring Buffer 256 byte ngắt nhận `USART1`, không chặn vòng điều khiển `10 ms`.
* Dùng `strtof()` để parse số thực ổn định trên `newlib-nano` (tránh lỗi `sscanf("%f")` bị vô hiệu hóa ở `nano.specs`), và hàm `format_float()` tự viết xử lý chuẩn dấu âm `-0.xxx`.

---

## 4. GIAO THỨC TRUYỀN THÔNG UART / BLE (115200 BPS)

* **Lệnh điều khiển & tinh chỉnh:** `F/B/L/R/S`, `AUTO/OFF`, `SET_PID:kp,ki,kd,skp,ski`, `GET_PID`, `OFFSET:<deg>`, `DEADZONE:<val>`, `SET_POLARITY:ml,mr,el,er`, `PARAMS`, `PROBE[:A|:B|:PWM]`, `MOTOR:l,r`, `RESET_ENC`, `START_STREAM/STOP_STREAM`, `PING`, `HELP`.
* **Luồng Telemetry 100 Hz (`Telemetry_SendData`):**
  ```text
  DATA:<pitch_deg>,<gyro_dps>,<pwm_avg>,<enc_left>,<enc_right>,<fsm_state>
  DBG:<target_angle_deg>,<speed_filtered_mps>,<speed_i_deg>,<pwm_left>,<pwm_right>
  ```

---

## 5. VÒNG LẶP ĐIỀU KHIỂN CHÍNH (`Core/Src/main.c`)

1. Khởi tạo ngoại vi HAL (`I2C1`, `TIM2`, `TIM3`, `TIM4`, `TIM10`, `USART1`), phục hồi I2C, khởi tạo & hiệu chuẩn `MPU6050`, khởi tạo `MotorEncoder`, `CascadePID`, `RobotFSM`, `Telemetry`.
2. Trong vòng lặp chính chờ cờ `g_control_loop_flag` từ ngắt `TIM10` (mỗi `10 ms`):
   * `Telemetry_ProcessCommands()` — xử lý lệnh UART/BLE không chặn.
   * `MPU6050_Update(0.01f)` — đọc IMU & cập nhật Kalman 1D.
   * `MotorEncoder_Update(0.01f)` — đọc xung encoder & tính vận tốc thực.
   * Nếu ở chế độ `SYS_MODE_AUTO`: gọi `RobotFSM_Update(...)` $\rightarrow$ `Motor_SetOutput(pwm_l, pwm_r)`.
   * Nếu bật streaming: gọi `Telemetry_SendData(...)` xuất cặp chuỗi `DATA:` + `DBG:`.
   * Cập nhật LED nhịp tim `PC13` (`1 Hz` bình thường, `5 Hz` mất MPU6050).

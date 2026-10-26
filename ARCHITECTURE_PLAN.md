# KIẾN TRÚC HỆ THỐNG PHẦN MỀM XE HAI BÁNH TỰ CÂN BẰNG
## (STM32F401CCU6 + PYTHON PIPELINE)

---

## 1. TỔNG QUAN HỆ THỐNG

Dự án xe hai bánh tự cân bằng (Self-Balancing Robot) bao gồm hai thành phần cốt lõi:
1. **Firmware nhúng thời gian thực (C/C++ STM32 HAL, build bằng CMake)**: Thực thi trên vi điều khiển STM32F401CCU6 (84 MHz, có FPU phần cứng) đảm nhiệm việc đọc cảm biến IMU MPU6050 qua I2C1 (400kHz), ước lượng góc nghiêng bằng bộ lọc Kalman, đọc xung encoder 2 bánh (TIM2/TIM3), điều khiển cầu H TB6612 bằng xung PWM (TIM4) và quản lý trạng thái xe (Cân bằng, Tự đứng dậy, Dừng an toàn) với chu kỳ cố định 10ms (100Hz) kích hoạt bởi ngắt TIM10.
2. **Bộ công cụ tự động hóa & mô phỏng (Python Tools)**: Thực thi trên máy tính, chịu trách nhiệm mô phỏng toán học phi tuyến, giải LQR con lắc ngược để sinh bộ PID ban đầu, thu thập dữ liệu qua UART/Bluetooth (hoặc dữ liệu giả lập có bù nhiễu), nhận dạng hàm truyền hệ thống (System Identification) và tối ưu hóa tham số Cascade PID.

```
+-----------------------------------------------------------------------------------+
|                                  MÁY TÍNH (PC)                                    |
|  +--------------------+     +-----------------------+     +--------------------+  |
|  | robot_config.yaml  | --> |   sim_and_init_pid    | --> | pid_AutoConfig.h   |  |
|  +--------------------+     +-----------------------+     +--------------------+  |
|                                         |                                         |
|  +--------------------+     +-----------v-----------+     +--------------------+  |
|  |  mock/real data    | --> |     auto_tune_pid     | --> | SET_PID (UART)     |  |
|  +--------------------+     +-----------------------+     +--------------------+  |
+-----------------------------------------|-----------------------------------------+
                                          | UART 115200bps (PA9/PA10)
+-----------------------------------------v-----------------------------------------+
|                               STM32F401CCU6 FIRMWARE                              |
|                                                                                   |
|  +-----------------------------------------------------------------------------+  |
|  |                     TIM10 Interrupt (100Hz / dt = 10ms)                     |  |
|  |  +------------------+     +------------------+     +---------------------+  |  |
|  |  |  mpu6050_filter  | --> |    cascade_pid   | --> |    motor_encoder    |  |  |
|  |  |  (Kalman Angle)  |     | (Angle PD/Vel PI)|     | (TB6612 PWM + DZone)|  |  |
|  |  +------------------+     +------------------+     +---------------------+  |  |
|  |            |                       ^                          |             |  |
|  |            +--------+     +--------+                          |             |  |
|  |                     v     |                                   v             |  |
|  |                  +-----------+                            +---------------+ |  |
|  |                  | robot_fsm | <------------------------- | TIM2/3 Encoder| |  |
|  |                  +-----------+                            +---------------+ |  |
|  +-----------------------------------------------------------------------------+  |
|                                          |                                         |
|  +---------------------------------------v-------------------------------------+  |
|  |          telemetry_uart (USART1 Ring Buffer + Command Parser + Stream)      |  |
|  +-----------------------------------------------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

---

## 2. CẤU TRÚC THƯ MỤC DỰ ÁN

Hệ thống được tổ chức phân tầng rõ ràng, tách bạch giữa mã nguồn nhúng của xe (`Modules/`), mã khởi tạo phần cứng CubeMX (`Core/`) và bộ công cụ máy tính (`Tools/`):

```
self_balancing_car/
├── CMakeLists.txt                      # CMake gốc cấu hình toàn bộ project
├── CMakePresets.json                   # Presets Debug / Release cho ARM GCC
├── STM32F401xx_FLASH.ld                # Linker script STM32F401CCU6 (256K Flash, 64K RAM)
├── startup_stm32f401xc.s               # Vector ngắt và khởi tạo stack/heap
├── ARCHITECTURE_PLAN.md                # Tài liệu kiến trúc & đặc tả API (file này)
├── README.md                           # Tài liệu tổng quan dự án & hướng dẫn sử dụng
├── .gitignore                          # Cấu hình bỏ qua file build và log tạm
│
├── cmake/
│   ├── gcc-arm-none-eabi.cmake         # Cấu hình Toolchain cross-compiler ARM
│   └── stm32cubemx/                    # CMake tự sinh từ CubeMX
│       └── CMakeLists.txt
│
├── Core/                               # Mã nguồn khởi tạo ngoại vi bởi CubeMX
│   ├── Inc/
│   │   ├── main.h                      # Khai báo định danh chân GPIO (AIN1/2, BIN1/2, STBY)
│   │   ├── gpio.h, i2c.h, tim.h, usart.h
│   │   └── stm32f4xx_it.h              # Nguyên mẫu ngắt
│   └── Src/
│       ├── main.c                      # Điểm vào chính, vòng lặp điều khiển 100Hz
│       ├── gpio.c, i2c.c, tim.c, usart.c
│       └── stm32f4xx_it.c              # Hàm ngắt TIM10 và USART1
│
├── Modules/                            # TOÀN BỘ MÃ NGUỒN ĐIỀU KHIỂN CỦA XE (C/C++)
│   ├── Inc/
│   │   ├── mpu6050_filter.h            # Đọc MPU6050 qua I2C1 và lọc Kalman góc nghiêng
│   │   ├── motor_encoder.h             # Đọc Encoder TIM2/3 và điều khiển TB6612 TIM4
│   │   ├── cascade_pid.h               # Thuật toán Cascade PID (Vòng góc + Vận tốc)
│   │   ├── robot_fsm.h                 # Máy trạng thái (BALANCING, RECOVERY, IDLE)
│   │   ├── telemetry_uart.h            # Giao thức truyền thông UART & Ring Buffer
│   │   └── pid_AutoConfig.h            # File sinh tự động chứa thông số PID khởi tạo
│   └── Src/
│       ├── mpu6050_filter.c
│       ├── motor_encoder.c
│       ├── cascade_pid.c
│       ├── robot_fsm.c
│       └── telemetry_uart.c
│
├── Tools/                              # BỘ CÔNG CỤ MÔ PHỎNG & TỐI ƯU HÓA PYTHON
│   ├── requirements.txt                # Danh sách thư viện Python cần cài đặt
│   ├── robot_config.yaml               # File thông số vật lý xe và ngưỡng FSM
│   ├── computed_gains.json             # File lưu trữ hệ số PID kèm mã băm MD5
│   ├── sim_and_init_pid.py             # LQR con lắc ngược & sinh pid_AutoConfig.h
│   ├── data_acquisition.py             # Giả lập Runge-Kutta & quét kết nối UART
│   ├── auto_tune_pid.py                # Nhận dạng hệ thống ARX & tối ưu Nelder-Mead
│   ├── test_control_sequence.py        # Test kịch bản tự động, vẽ biểu đồ 3 kênh
│   └── esp32_s3_ble_bridge/            # Firmware nạp cho module Bluetooth rời ESP32-S3 (BLE 5.0)
│
├── setup.sh / setup.bat                # Tự động hóa cài đặt môi trường ảo Python
├── build_firmware.sh / build_firmware.bat  # Tự động hóa build firmware C/C++
└── run_pipeline.sh / run_pipeline.bat      # Tự động hóa điều phối chuỗi công cụ mô phỏng & tối ưu
```

---

## 3. THIẾT KẾ CHI TIẾT TỪNG MODULE & ĐẶC TẢ GIAO DIỆN API

### 3.1 Module Cảm Biến IMU & Lọc Kalman (`mpu6050_filter`)
- **Vai trò**: Giao tiếp I2C1 (PB6/PB7 @ 400kHz), đọc gia tốc kế 3 trục và con quay hồi chuyển 3 trục. Hiệu chuẩn bù trôi lúc khởi động và sử dụng bộ lọc Kalman 1 chiều để tính toán chính xác góc nghiêng `pitch_angle` (đơn vị: độ) và vận tốc góc `gyro_rate` (đơn vị: độ/giây).
- **Cấu trúc dữ liệu chính**:
  ```c
  typedef struct {
      float pitch;          // Góc nghiêng ước lượng sau lọc Kalman (độ)
      float gyro_rate;      // Vận tốc góc theo trục pitch (độ/giây)
      float ax_raw, ay_raw, az_raw;
      float gx_raw, gy_raw, gz_raw;
      float gyro_bias;      // Độ trôi con quay ước lượng bởi Kalman
      uint8_t is_calibrated;
  } mpu6050_data_t;
  ```
- **Giao diện API (`mpu6050_filter.h`)**:
  ```c
  // Khởi tạo MPU6050: wake up, set DLPF 42Hz, range gyro +/-2000dps, accel +/-4g
  HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

  // Routine hiệu chuẩn bù trôi bias (lấy mẫu 300 lần ở trạng thái đứng yên)
  void MPU6050_Calibrate(void);

  // Cập nhật bộ lọc Kalman (được gọi định kỳ mỗi dt = 0.01s trong vòng ngắt 100Hz)
  void MPU6050_Update(float dt);

  // Lấy dữ liệu góc nghiêng và vận tốc góc hiện tại
  float MPU6050_GetPitch(void);
  float MPU6050_GetGyroRate(void);
  const mpu6050_data_t* MPU6050_GetData(void);
  ```

---

### 3.2 Module Động Cơ TB6612 & Encoder (`motor_encoder`)
- **Vai trò**:
  - Đọc Encoder 2 bánh xe từ TIM2 (Trái) và TIM3 (Phải) ở chế độ 4x Encoder Mode, xử lý chống tràn thanh ghi 16-bit (ARR=65535), tính toán quãng đường và vận tốc thực (m/s).
  - Điều khiển cầu H TB6612FNG qua xung PWM TIM4_CH3 (PB8) và TIM4_CH4 (PB9) ở tần số 20kHz, các chân GPIO điều khiển chiều quay (PA4, PA5, PA6, PA7) và chân kích hoạt PB0 (STBY).
  - Tự động bù vùng chết tĩnh cơ khí (`pwm_deadzone`).
- **Cấu trúc dữ liệu chính**:
  ```c
  typedef struct {
      int32_t total_pulses_left;
      int32_t total_pulses_right;
      float speed_left_mps;     // Vận tốc bánh trái (m/s)
      float speed_right_mps;    // Vận tốc bánh phải (m/s)
      float speed_avg_mps;      // Vận tốc trung bình (m/s)
      int16_t pwm_left;         // Giá trị PWM hiện thời (-4199 đến 4199)
      int16_t pwm_right;
      uint16_t deadzone;        // Vùng chết (mặc định 450)
  } motor_encoder_data_t;
  ```
- **Giao diện API (`motor_encoder.h`)**:
  ```c
  // Khởi tạo PWM TIM4, bật Encoder TIM2, TIM3, kéo PB0 (STBY) lên HIGH
  void MotorEncoder_Init(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_enc_l, TIM_HandleTypeDef *htim_enc_r);

  // Đọc xung encoder, tính vận tốc (chu kỳ dt = 0.01s)
  void MotorEncoder_Update(float dt);

  // Đặt tốc độ và chiều quay cho 2 bánh xe (nhận giá trị từ -4199 đến +4199)
  // Tự động xử lý bù vùng chết pwm_deadzone và bão hòa
  void Motor_SetOutput(int16_t left_pwm, int16_t right_pwm);

  // Ngắt an toàn động cơ (hạ STBY xuống LOW, ngắt PWM về 0)
  void Motor_Disable(void);

  // Bật lại động cơ (kéo STBY lên HIGH)
  void Motor_Enable(void);

  // Thiết lập bù vùng chết động cơ
  void Motor_SetDeadzone(uint16_t deadzone);

  // Lấy dữ liệu đo đạc encoder
  const motor_encoder_data_t* MotorEncoder_GetData(void);
  ```

---

### 3.3 Module Điều Khiển Cascade PID (`cascade_pid`)
- **Vai trò**: Cung cấp cấu trúc điều khiển nối tầng gồm hai vòng:
  1. **Vòng ngoài (Velocity Loop - PI)**: So sánh sai số giữa vận tốc mục tiêu $v_{target}$ và vận tốc thực $v_{meas}$, tạo ra góc nghiêng mục tiêu $\theta_{target}$ (giới hạn trong khoảng $\pm 10^\circ$).
  2. **Vòng trong (Angle Loop - PD/PID)**: So sánh sai số giữa $\theta_{target}$ và góc đo $\theta_{meas}$. Sử dụng trực tiếp vận tốc góc từ Gyroscope ($-\dot{\theta}$) cho khâu vi phân để loại bỏ triệt để hiện tượng vọt lố Derivative Kick, tính ra xung lực điều khiển PWM.
  3. **Khâu chống bão hòa tích phân (Anti-windup)**: Ngăn hiện tượng tràn bộ tích phân khi động cơ chạm ngưỡng công suất tối đa.
- **Cấu trúc dữ liệu chính**:
  ```c
  typedef struct {
      // Thông số vòng góc
      float angle_kp;
      float angle_ki;
      float angle_kd;
      float angle_integral;

      // Thông số vòng vận tốc
      float speed_kp;
      float speed_ki;
      float speed_integral;

      // Giới hạn
      float max_target_angle;   // Mặc định 10.0 độ
      int16_t max_pwm_output;   // Mặc định 4199
      float integral_limit;
  } cascade_pid_t;
  ```
- **Giao diện API (`cascade_pid.h`)**:
  ```c
  // Khởi tạo bộ điều khiển với các thông số mặc định từ pid_AutoConfig.h
  void CascadePID_Init(void);

  // Xóa toàn bộ tích phân khi chuyển đổi trạng thái FSM
  void CascadePID_Reset(void);

  // Cập nhật nóng thông số PID trong RAM
  void CascadePID_SetGains(float ang_kp, float ang_ki, float ang_kd, float spd_kp, float spd_ki);

  // Tính toán Cascade PID:
  // - target_speed (m/s): Vận tốc mong muốn (từ lệnh Bluetooth)
  // - measured_speed (m/s): Vận tốc đo từ Encoder
  // - measured_angle (độ): Góc nghiêng từ Kalman
  // - gyro_rate (độ/giây): Vận tốc góc từ Gyro
  // - turn_offset: Độ lệch công suất 2 bánh khi quay xe
  // - out_pwm_left, out_pwm_right: Giá trị PWM xuất ra động cơ
  void CascadePID_Compute(float target_speed, float measured_speed,
                          float measured_angle, float gyro_rate,
                          float turn_offset, float dt,
                          int16_t *out_pwm_left, int16_t *out_pwm_right);
  ```

---

### 3.4 Module Máy Trạng Thái Hữu Hạn (`robot_fsm`)
- **Vai trò**: Quản lý hành vi và trạng thái vận hành của xe, đảm bảo an toàn tuyệt đối và tính thông minh trong các trường hợp nghiêng ngã:
  1. `STATE_BALANCING`: Góc nghiêng $|\theta| \le \theta_{normal}$ ($20^\circ$). Xe cân bằng ổn định. Nếu nhận lệnh di chuyển từ Bluetooth (`FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`), áp dụng thêm offset vận tốc/góc. Khi buông tay (không có lệnh), tự động phanh vận tốc về 0 m/s để xe đứng cân bằng tại chỗ.
  2. `STATE_RECOVERY`: Góc nghiêng $\theta_{normal} < |\theta| \le \theta_{max\_recovery}$ ($20^\circ < |\theta| \le 45^\circ$). Kích hoạt chuỗi xung lực gia tốc Swing-up cực đại cùng chiều nghiêng để hất trọng tâm xe dựng đứng dậy. Khi góc trở về $\le \theta_{normal}$, reset tích phân PID và chuyển mượt sang `STATE_BALANCING`. Nếu sau timeout 1.5s chưa dựng được, chuyển về `STATE_IDLE`.
  3. `STATE_IDLE`: Góc nghiêng $|\theta| > \theta_{max\_recovery}$ ($> 45^\circ$). Xe đã ngã đổ hoàn toàn. Lập tức ngắt PWM = 0, kéo `PB0` (STBY) xuống LOW để bảo vệ động cơ và pin, xóa bộ nhớ tích phân PID. Tự động chuyển về trạng thái sẵn sàng khi xe được nhấc đứng dậy.
- **Cấu trúc dữ liệu chính**:
  ```c
  typedef enum {
      STATE_IDLE = 0,
      STATE_BALANCING = 1,
      STATE_RECOVERY = 2
  } robot_state_t;

  typedef enum {
      CMD_NONE = 0,
      CMD_FORWARD,
      CMD_BACKWARD,
      CMD_TURN_LEFT,
      CMD_TURN_RIGHT,
      CMD_STOP
  } robot_cmd_t;

  typedef struct {
      robot_state_t state;
      robot_cmd_t current_cmd;
      float target_speed;
      float turn_offset;
      uint16_t recovery_timer_ticks;
      float theta_normal;       // 20.0 độ
      float theta_max_recovery; // 45.0 độ
  } robot_fsm_t;
  ```
- **Giao diện API (`robot_fsm.h`)**:
  ```c
  void RobotFSM_Init(float theta_normal, float max_recovery_angle);
  void RobotFSM_SetCommand(robot_cmd_t cmd);
  void RobotFSM_Update(float angle, float gyro_rate, float speed, float dt,
                       int16_t *cmd_pwm_left, int16_t *cmd_pwm_right);
  robot_state_t RobotFSM_GetState(void);
  const char* RobotFSM_GetStateString(void);
  ```

---

### 3.5 Module Truyền Thông Telemetry & Lệnh UART (`telemetry_uart`)
- **Vai trò**:
  - Giao tiếp với module Bluetooth ESP32-S3 DevKitC (BLE 5.0) và PC qua `USART1` (PA9 TX, PA10 RX) ở Baudrate 115200bps.
  - Sử dụng ngắt nhận `HAL_UART_Receive_IT` kết hợp Ring Buffer để không bao giờ làm chặn hoặc chậm trễ vòng lặp điều khiển 100Hz.
  - Phân tích cú pháp các lệnh điều khiển, bắt tay PING/PONG, lệnh nạp nóng PID `SET_PID:...`, và phát chuỗi dữ liệu Telemetry định kỳ.
- **Cấu trúc dữ liệu chính**:
  ```c
  #define UART_RX_BUFFER_SIZE 256

  typedef struct {
      uint8_t buffer[UART_RX_BUFFER_SIZE];
      volatile uint16_t head;
      volatile uint16_t tail;
  } uart_ringbuf_t;
  ```
- **Giao diện API (`telemetry_uart.h`)**:
  ```c
  void Telemetry_Init(UART_HandleTypeDef *huart);
  void Telemetry_OnRxByte(uint8_t byte);   // Gọi trong ngắt USART1_IRQHandler
  void Telemetry_ProcessIncoming(void);    // Gọi trong vòng lặp chính để xử lý lệnh
  void Telemetry_SendData(float angle, float gyro, int16_t pwm,
                          int32_t enc_left, int32_t enc_right, uint8_t state);
  void Telemetry_SendString(const char *str);
  uint8_t Telemetry_IsStreaming(void);
  ```

---

## 4. ĐẶC TẢ GIAO THỨC TRUYỀN THÔNG UART (TELEMETRY PROTOCOL)

Tốc độ: **115200 bps**, Khung: **8 Data bits, No Parity, 1 Stop bit (8N1)**. Mọi gói tin kết thúc bằng ký tự xuống dòng `\n`.

### 4.1 Chiều Máy Tính / Điện Thoại $\rightarrow$ STM32
| Gói tin (ASCII) | Mục đích | Phản hồi từ STM32 |
| :--- | :--- | :--- |
| `PING\n` | Kiểm tra kết nối phần cứng và cảm biến | `PONG\n` |
| `SET_PID:<ang_kp>,<ang_ki>,<ang_kd>,<spd_kp>,<spd_ki>\n` | Cập nhật nóng 5 hệ số PID trong RAM | `OK_PID\n` |
| `CMD:FORWARD\n` hoặc `F` | Lệnh di chuyển tiến | `OK_CMD\n` |
| `CMD:BACKWARD\n` hoặc `B` | Lệnh di chuyển lùi | `OK_CMD\n` |
| `CMD:LEFT\n` hoặc `L` | Lệnh quay trái | `OK_CMD\n` |
| `CMD:RIGHT\n` hoặc `R` | Lệnh quay phải | `OK_CMD\n` |
| `CMD:STOP\n` hoặc `S` | Dừng lệnh, tự động khóa phanh về cân bằng | `OK_CMD\n` |
| `START_STREAM\n` | Bật luồng gửi dữ liệu đo đạc thời gian thực | `STREAM_STARTED\n` |
| `STOP_STREAM\n` | Tắt luồng gửi dữ liệu | `STREAM_STOPPED\n` |

### 4.2 Chiều STM32 $\rightarrow$ Máy Tính (Telemetry Stream)
Định dạng bản tin chu kỳ 10ms (khi bật stream):
```
DATA:<angle>,<gyro>,<pwm>,<enc_left>,<enc_right>,<state>\n
```
- `<angle>`: Góc nghiêng pitch hiện tại (float, ví dụ `1.25`)
- `<gyro>`: Vận tốc góc theo trục pitch (float, ví dụ `-0.12`)
- `<pwm>`: Xung PWM trung bình cấp cho 2 động cơ (int, dải -4199 đến 4199)
- `<enc_left>`: Xung encoder tích lũy bánh trái (int32)
- `<enc_right>`: Xung encoder tích lũy bánh phải (int32)
- `<state>`: Trạng thái FSM (int: 0 = IDLE, 1 = BALANCING, 2 = RECOVERY)

Ví dụ gói tin mẫu:
```
DATA:0.42,-0.15,350,1240,1238,1\n
```

---

## 5. TÍCH HỢP HỆ THỐNG VÀO VÒNG LẶP ĐIỀU KHIỂN CHÍNH (Core Integration)

- **Ngắt TIM10 (100Hz / chu kỳ 10ms)**:
  ```c
  // Trong stm32f4xx_it.c hoặc callback:
  void TIM1_UP_TIM10_IRQHandler(void) {
      HAL_TIM_IRQHandler(&htim10);
      system_tick_10ms_flag = 1;
  }
  ```
- **Vòng lặp trong `main.c`**:
  ```c
  while (1) {
      if (system_tick_10ms_flag) {
          system_tick_10ms_flag = 0;

          // 1. Đọc IMU và lọc Kalman góc nghiêng
          MPU6050_Update(0.01f);

          // 2. Đọc Encoder và tính vận tốc bánh xe
          MotorEncoder_Update(0.01f);

          // 3. Xử lý lệnh nhận từ UART
          Telemetry_ProcessIncoming();

          // 4. Cập nhật Máy trạng thái FSM & Cascade PID
          int16_t pwm_left = 0, pwm_right = 0;
          RobotFSM_Update(MPU6050_GetPitch(), MPU6050_GetGyroRate(),
                          MotorEncoder_GetData()->speed_avg_mps,
                          0.01f, &pwm_left, &pwm_right);

          // 5. Xuất xung PWM ra mạch cầu H TB6612
          Motor_SetOutput(pwm_left, pwm_right);

          // 6. Gửi Telemetry nếu đang ở chế độ stream
          if (Telemetry_IsStreaming()) {
              Telemetry_SendData(MPU6050_GetPitch(), MPU6050_GetGyroRate(),
                                 (pwm_left + pwm_right) / 2,
                                 MotorEncoder_GetData()->total_pulses_left,
                                 MotorEncoder_GetData()->total_pulses_right,
                                 RobotFSM_GetState());
          }
      }
  }
  ```
  > Toàn bộ logic trên chạy hoàn toàn đồng bộ, không tồn tại bất kỳ lời gọi hàm trễ `HAL_Delay()` nào trong chu trình điều khiển.

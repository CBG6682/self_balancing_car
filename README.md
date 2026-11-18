# Hệ Thống Xe Hai Bánh Tự Cân Bằng (STM32F401CCU6 Black Pill)
## Dual-Loop Cascade PID (Soft-Zone & Short-Brake), 1D Kalman Filter, Robot FSM & Python Toolchain

[![STM32](https://img.shields.io/badge/MCU-STM32F401CCU6-002B49?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f401cc.html)
[![Toolchain](https://img.shields.io/badge/Toolchain-Arm_GNU_GCC_%2B_CMake-blue?logo=cmake)](https://cmake.org/)
[![BLE](https://img.shields.io/badge/Wireless-ESP32--S3_BLE_5.0-E7352C?logo=espressif)](Tools/esp32_s3_ble_bridge/README.md)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python)](https://www.python.org/)

Dự án xây dựng hoàn chỉnh firmware nhúng thời gian thực và bộ công cụ kiểm thử – hiệu chuẩn – tinh chỉnh bằng Python cho **Xe Hai Bánh Tự Cân Bằng (Two-Wheeled Self-Balancing Robot)** sử dụng vi điều khiển **STM32F401CCU6** (ARM Cortex-M4 @ 84 MHz), cảm biến **MPU6050**, mạch cầu H **TB6612FNG**, cặp động cơ **GA25 (21.3:1, 937 CPR)** và mạch cầu nối **Bluetooth BLE 5.0 ESP32-S3**.

---

## MỤC LỤC
1. [Tính Năng & Thông Số Tối Ưu Cuối Cùng](#1-tính-năng--thông-số-tối-ưu-cuối-cùng)
2. [Sơ Đồ Đấu Nối Phần Cứng (Hardware Pinout)](#2-sơ-đồ-đấu-nối-phần-cứng-hardware-pinout)
3. [Kiến Trúc Phần Mềm](#3-kiến-trúc-phần-mềm)
4. [Cấu Trúc Thư Mục Dự Án](#4-cấu-trúc-thư-mục-dự-án)
5. [Cài Đặt Môi Trường, Biên Dịch & Nạp Firmware](#5-cài-đặt-môi-trường-biên-dịch--nạp-firmware)
6. [Hướng Dẫn Bộ Công Cụ Python (Tools)](#6-hướng-dẫn-bộ-công-cụ-python-tools)
7. [Bảng Lệnh Bluetooth BLE & Giao Thức Telemetry UART](#7-bảng-lệnh-bluetooth-ble--giao-thức-telemetry-uart)

---

## 1. TÍNH NĂNG & THÔNG SỐ TỐI ƯU CUỐI CÙNG

### Kiến trúc điều khiển & cơ chế mô phỏng "Đệm ảo" (Virtual Mattress)
* **Cascade PID 2 vòng @ 100 Hz (TIM10, `dt = 10 ms`)**:
  * **Vòng trong (Angle PD + Soft-Zone):** Ổn định góc nghiêng thân xe. Trong vùng `|angle_error| < 1.2°`, độ cứng `Kp` giảm mềm còn **60%** (`~24`) rồi tăng tuyến tính mượt lên **100%** (`40.0`) khi gặp ngoại lực, giúp xe đứng êm không giật bánh răng mà vẫn đủ lực đỡ khi bị đẩy. Khâu vi phân dùng trực tiếp `gyro_rate` qua bộ lọc IIR `0.60 / 0.40` ($f_c \approx 10.6\text{ Hz}$), không deadband để giảm chấn liên tục xuống tới `0 dps`.
  * **Vòng ngoài (Velocity/Position PI tách nhánh):** Lọc tốc độ `0.85 / 0.15` ($f_c \approx 2.8\text{ Hz}$). Nhánh **P** luôn hãm vận tốc thực (`-v_active`) với làm mềm bậc hai trong `±0.02 m/s` để vừa bù phản điện động (Back-EMF) của động cơ GA25 vừa tránh nhiễu 1 xung encoder. Nhánh **I** tích phân không deadband (`target_speed - v_raw`, kẹp động `±2.5°`) giúp neo giữ vị trí đứng yên và dẫn xe chạy tiến/lùi êm.
* **Cơ cấu chấp hành TB6612FNG & Encoder 4x (`ARR = 999`, `21 kHz`):**
  * **Deadzone riêng từng bánh** (`PWM_DEADZONE_LEFT = 23`, `PWM_DEADZONE_RIGHT = 17`, ramp mượt `DEADZONE_SMOOTH_THRESHOLD = 10`) và **hệ số cân bằng lực 2 bánh** (`MOTOR_LEFT_SCALE = 1.00`, `MOTOR_RIGHT_SCALE = 0.93`).
  * **Short-Brake thụ động (`IN1 = HIGH, IN2 = HIGH`)** trong vùng chết `|pwm| <= 2`, tạo lực cản lăn điện từ thụ động giúp xe bám chắc tại điểm cân bằng trên nền gạch nhẵn.
  * **Giới hạn tải an toàn 80%** (`PWM_MAX_LIMIT = 800`) và **Slew-rate** (`40 PWM/tick`) tránh dòng đỉnh đột ngột.
* **Lọc cảm biến MPU6050 (I2C1 400 kHz, DLPF 42 Hz):**
  * Hiệu chuẩn 300 mẫu bias con quay khi khởi động (không trừ nhầm góc nghiêng gia tốc kế), dung hòa bằng **Bộ lọc Kalman 1D**.
* **Máy trạng thái hữu hạn FSM (`robot_fsm.c`):**
  * `STATE_IDLE`: Ngắt động cơ (`STBY = PA2 LOW`, `PWM = 0`) khi nghiêng quá `±25°` hoặc quá tải `≥ 80%` liên tục `100 ms`. Chỉ tự tái kích hoạt khi dựng thẳng trong `±10°` đủ `200 ms`.
  * `STATE_BALANCING`: Cân bằng & nhận lệnh lái từ BLE với **ramp tốc độ mượt** (`RC_MOVE_SPEED_MPS = 0.15 m/s`, `0.004/tick`), **watchdog 500 ms** tự dừng khi mất lệnh.

### Bộ tham số tối ưu cuối cùng (đã lưu cứng trong [`Modules/Inc/pid_AutoConfig.h`](Modules/Inc/pid_AutoConfig.h))

| Tham số | Giá trị | Ý nghĩa |
| :--- | :--- | :--- |
| `DEFAULT_ANGLE_KP` / `KI` / `KD` | `40.00` / `0.000` / `1.400` | Bộ PD vòng góc (Soft-Zone `60% → 100%` quanh `±1.2°`) |
| `DEFAULT_SPEED_KP` / `KI` | `28.00` / `2.500` | Vòng ngoài hãm tốc, bù Back-EMF & neo vị trí |
| `MECHANICAL_BALANCE_OFFSET_DEG` | `-3.60°` | Góc cân bằng trọng tâm cơ khí |
| `PWM_TIMER_ARR` / `PWM_MAX_LIMIT` | `999` (`21 kHz`) / `800` | Thang PWM `0–1000`, bảo vệ trần tải `80%` |
| `PWM_DEADZONE_LEFT` / `RIGHT` | `23` / `17` | Bù vùng chết có tải riêng từng bánh (~85% ngưỡng thật `27 / 20`) |
| `MOTOR_LEFT_SCALE` / `RIGHT_SCALE` | `1.00` / `0.93` | Cân bằng tốc độ 2 động cơ GA25 |
| `ENCODER_CPR` | `937` | GA25 tỷ số `21.3:1` ($11 \times 21.3 \times 4 = 937.2$ xung/vòng) |
| `RC_MOVE_SPEED_MPS` / `RC_TURN_OFFSET_PWM` | `0.15 m/s` / `150` | Tốc độ lái tiến/lùi êm (có ramp) & độ lệch quay trái/phải |

---

## 2. SƠ ĐỒ ĐẤU NỐI PHẦN CỨNG (HARDWARE PINOUT)

Khớp 100% với cấu hình CubeMX (`Core/Inc/main.h`, `Core/Src/gpio.c`, `tim.c`, `i2c.c`, `usart.c`) và firmware ESP32-S3 (`Tools/esp32_s3_ble_bridge/esp32_s3_ble_bridge.ino`):

| Phân Hệ | Chân Module | Chân STM32 | Ngoại Vi / Chức Năng | Điện Áp |
| :--- | :--- | :--- | :--- | :--- |
| **MPU6050** | VCC / GND | 3.3V / GND | Nguồn cảm biến IMU | 3.3V / 0V |
| | SCL | **PB6** | `I2C1_SCL` (Fast Mode 400 kHz) | 3.3V |
| | SDA | **PB7** | `I2C1_SDA` | 3.3V |
| **TB6612FNG** | VM | LiPo 3S (+) | Nguồn động cơ GA25 | 11.1V – 12.6V |
| | VCC / GND | 3.3V / GND | Nguồn logic cầu H (Mass chung với Pin & STM32) | 3.3V / 0V |
| | STBY | **PA2** | `GPIO_Output` (HIGH = Bật cầu H, LOW = Ngắt an toàn) | 3.3V Logic |
| | PWMA | **PB8** | `TIM4_CH3` (PWM 21 kHz, `ARR = 999` — bánh trái) | 3.3V Logic |
| | PWMB | **PB9** | `TIM4_CH4` (PWM 21 kHz, `ARR = 999` — bánh phải) | 3.3V Logic |
| | AIN1 | **PA4** | `GPIO_Output` (Chiều động cơ trái A1) | 3.3V Logic |
| | AIN2 | **PA5** | `GPIO_Output` (Chiều động cơ trái A2) | 3.3V Logic |
| | BIN1 | **PA3** | `GPIO_Output` (Chiều động cơ phải B1) | 3.3V Logic |
| | BIN2 | **PA7** | `GPIO_Output` (Chiều động cơ phải B2) | 3.3V Logic |
| | AO1, AO2 / BO1, BO2 | Motor Trái / Phải | Ngõ ra cấp điện 2 động cơ GA25 | Tới 12V PWM |
| **Encoder Trái** | VCC / GND | 3.3V / GND | Nguồn cảm biến Hall | 3.3V / 0V |
| | Phase A / B | **PA0** / **PA1** | `TIM2_CH1` / `TIM2_CH2` (4x Encoder Mode) | 3.3V |
| **Encoder Phải** | VCC / GND | 3.3V / GND | Nguồn cảm biến Hall | 3.3V / 0V |
| | Phase A / B | **PB4** / **PB5** | `TIM3_CH1` / `TIM3_CH2` (4x Encoder Mode) | 3.3V |
| **ESP32-S3 BLE Bridge** | 5V / GND | 5V Buck / GND | Nguồn Buck 5V (≥ 500 mA), nối chung GND | 5V / 0V |
| | **GPIO 6** (`UART1_TX`) | **PA10** | `USART1_RX` (Lệnh từ BLE / PC → STM32, 115200 bps) | 3.3V Logic |
| | **GPIO 3** (`UART1_RX`) | **PA9** | `USART1_TX` (Telemetry từ STM32 → BLE / PC, 115200 bps) | 3.3V Logic |
| **Nạp ST-Link SWD** | SWDIO / SWCLK | **PA13** / **PA14** | Giao tiếp nạp & debug SWD | 3.3V |
| **Đèn trạng thái** | Onboard LED | **PC13** | Active-LOW (`1 Hz` bình thường, `5 Hz` lỗi MPU6050) | 3.3V |
| **Định thời vòng lặp** | Nội bộ | — | **TIM10** (Ngắt định kỳ `10 ms` / `100 Hz`) | Nội bộ |

> [!WARNING]
> Cực âm (`GND`) của **Pin LiPo 3S**, **STM32 BlackPill**, **TB6612FNG**, **MPU6050** và **ESP32-S3** bắt buộc nối chung điểm mass.

---

## 3. KIẾN TRÚC PHẦN MỀM

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

## 4. CẤU TRÚC THƯ MỤC DỰ ÁN

```text
self_balancing_car/
├── CMakeLists.txt              # Cấu hình build CMake đa nền tảng
├── CMakePresets.json           # Presets Debug / Release (Ninja)
├── setup.sh / setup.bat        # Tự động tạo Python venv & cài thư viện
├── build_firmware.sh / .bat    # Biên dịch firmware ra .elf, .bin, .hex
├── run_pipeline.sh / .bat      # Menu khởi chạy nhanh toàn bộ công cụ Python
├── ARCHITECTURE_PLAN.md        # Tài liệu thiết kế kiến trúc phần mềm chi tiết
├── Core/                       # Mã nguồn khởi tạo ngoại vi STM32CubeMX
├── Drivers/                    # Thư viện CMSIS & STM32F4xx HAL
├── Modules/                    # Thư viện điều khiển xe tự cân bằng
│   ├── Inc/
│   │   ├── pid_AutoConfig.h    # Tập trung toàn bộ hằng số PID, Deadzone, Offset, Polarity
│   │   ├── cascade_pid.h       # API Cascade PID (Soft-Zone & tách nhánh P/I)
│   │   ├── mpu6050_filter.h    # API driver MPU6050 I2C1 & Kalman 1D
│   │   ├── motor_encoder.h     # API điều khiển TB6612, Short-Brake, Probe & Encoder 4x
│   │   ├── robot_fsm.h         # API máy trạng thái FSM & ramp tốc độ lái
│   │   ├── telemetry_uart.h    # API Ring Buffer UART1 & parser lệnh BLE
│   │   ├── motor_test.h        # API chu trình test động cơ
│   │   └── uart_loopback_test.h
│   └── Src/                    # Hiện thực tương ứng các module trên
└── Tools/                      # Bộ công cụ kiểm thử, hiệu chuẩn, nạp & tinh chỉnh
    ├── MODULE_TEST_GUIDE.md    # Hướng dẫn kiểm thử từng linh kiện & đo VOM
    ├── robot_config.yaml       # Thông số vật lý khung xe, động cơ & PID mặc định
    ├── requirements.txt        # Thư viện Python (pyserial, numpy, scipy, matplotlib, pyyaml)
    ├── logger_utils.py         # Tiện ích log màu & lọc cổng serial
    ├── flash_stlink.sh/.py/.bat# Nạp firmware STM32 qua ST-Link V2/V3 (SWD)
    ├── flash_usb.sh / flash_hid.py # Nạp firmware qua USB HID Bootloader
    ├── setup_udev.sh           # Cấu hình quyền cổng USB / ST-Link trên Linux
    ├── check_connection.py     # Quét & chẩn đoán kết nối USB-UART / ESP32 Bridge
    ├── system_self_test.py     # Kiểm tra tổng thể từng linh kiện có xác nhận
    ├── live_sensor_calib.py    # Kiểm tra & hiệu chuẩn cảm biến MPU6050, Encoder
    ├── find_deadzone.py        # Đo tự động ngưỡng vùng chết từng động cơ GA25
    ├── find_zero_balance.py    # Đo góc cân bằng trọng tâm cơ khí (Offset)
    ├── live_balance_tuning.py  # Tinh chỉnh PID & theo dõi trực tiếp khi xe chạy
    ├── esp32_tune_monitor.py   # Terminal giám sát & gửi lệnh nóng qua ESP32 Bridge
    ├── record_session.py       # Ghi phiên telemetry 100Hz (DATA + DBG) ra CSV
    ├── sim_and_init_pid.py     # Mô phỏng con lắc ngược & giải LQR khởi tạo PID
    ├── auto_tune_pid.py        # Nhận dạng hệ thống ARX & tối ưu Nelder-Mead ITAE
    └── esp32_s3_ble_bridge/    # Firmware BLE 5.0 cho mạch cầu nối ESP32-S3
        ├── esp32_s3_ble_bridge.ino
        └── README.md
```

---

## 5. CÀI ĐẶT MÔI TRƯỜNG, BIÊN DỊCH & NẠP FIRMWARE

### 5.1. Cài đặt môi trường
* **Linux (Ubuntu / Debian / Arch):**
  ```bash
  sudo apt update
  sudo apt install -y gcc-arm-none-eabi cmake ninja-build stlink-tools python3 python3-venv python3-pip
  ./setup.sh
  ```
* **Windows:** Cài Arm GNU Toolchain, CMake, Python 3.8+ (thêm vào `PATH`), sau đó chạy `setup.bat`.

### 5.2. Biên dịch firmware
```bash
# Linux / macOS:
./build_firmware.sh
# hoặc biên dịch tăng cường nhanh bằng Ninja:
ninja -C build/Debug

# Windows:
build_firmware.bat
```
File nhị phân đầu ra: `build/Debug/self_balancing_car.bin` (cùng `.elf`, `.hex`).

### 5.3. Nạp firmware vào STM32
* **Qua ST-Link V2/V3 (khuyến nghị):** Nối `SWDIO (PA13)`, `SWCLK (PA14)`, `GND`, `3.3V` rồi chạy:
  ```bash
  ./Tools/flash_stlink.sh          # Linux / macOS
  Tools\flash_stlink.bat           # Windows
  ```
* **Qua USB HID Bootloader (nếu mạch có WeAct HID Bootloader):**
  ```bash
  ./Tools/flash_usb.sh
  ```

---

## 6. HƯỚNG DẪN BỘ CÔNG CỤ PYTHON (`Tools/`)

Bạn có thể mở menu tương tác chứa toàn bộ công cụ bằng:
```bash
./run_pipeline.sh     # Linux / macOS
run_pipeline.bat      # Windows
```

Hoặc gọi trực tiếp từng công cụ theo đúng quy trình kiểm thử & hiệu chuẩn:

| Bước | Công cụ | Lệnh chạy nhanh | Mục đích |
| :---: | :--- | :--- | :--- |
| **1** | `check_connection.py` | `./run_pipeline.sh -c` | Quét cổng `/dev/ttyACM*`, kiểm tra bắt tay `PING`/`ESP:PING` với ESP32-S3 & STM32 |
| **2** | `system_self_test.py` | `./run_pipeline.sh -d` | Kiểm tra tổng thể từng phần cứng: UART, MPU6050, Encoder trái/phải, cầu H TB6612 & 2 động cơ |
| **3** | `live_sensor_calib.py` | `./run_pipeline.sh -s` | Kiểm tra chiều góc Pitch, tốc độ Gyro và chiều đếm xung Encoder 2 bánh theo thời gian thực |
| **4** | `find_deadzone.py` | `./run_pipeline.sh -dz` | Tăng dần PWM từ 0 để tìm ngưỡng bắt đầu quay có tải của từng động cơ Trái/Phải |
| **5** | `find_zero_balance.py` | `./run_pipeline.sh -o` | Đo góc nghiêng trọng tâm tĩnh khi dựng cân bằng xe để xác định `MECHANICAL_BALANCE_OFFSET_DEG` |
| **6** | `sim_and_init_pid.py` | `./run_pipeline.sh --sim` | Giải LQR trên mô hình vật lý từ `Tools/robot_config.yaml` để có bộ PID khởi điểm |
| **7** | `esp32_tune_monitor.py` | `./run_pipeline.sh -m` | Màn hình giám sát & gửi lệnh nóng `SET_PID`, `OFFSET`, `DEADZONE`, `AUTO/OFF` qua cổng nối tiếp |
| **8** | `live_balance_tuning.py` | `./run_pipeline.sh -t` | Công cụ tinh chỉnh PID trực tiếp theo từng vòng khi xe đang chạy thực tế |
| **9** | `record_session.py` | `./run_pipeline.sh -r` | Ghi phiên dữ liệu 100 Hz (`DATA:` + `DBG:`) để phân tích dao động và đáp ứng tốc độ |
| **10** | `auto_tune_pid.py` | `./run_pipeline.sh --autotune` | Nhận dạng mô hình ARX(2,2) & tối ưu hóa PID theo chỉ số ITAE (Nelder-Mead) |

Chi tiết kiểm thử từng linh kiện và đo VOM xem tại [`Tools/MODULE_TEST_GUIDE.md`](Tools/MODULE_TEST_GUIDE.md).

---

## 7. BẢNG LỆNH BLUETOOTH BLE & GIAO THỨC TELEMETRY UART

* **UART1:** `115200 bps`, 8N1 (`PA9/PA10`).
* **ESP32-S3 BLE 5.0 (Nordic UART Service — NUS):**
  * Tên thiết bị: `STM32_Balance_Car`
  * Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
  * RX (App → Xe): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
  * TX (Xe → App): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
  * App khuyến nghị: *Serial Bluetooth Terminal* (Android) hoặc *nRF Connect / BLE Terminal* (iOS). Xem thêm tại [`Tools/esp32_s3_ble_bridge/README.md`](Tools/esp32_s3_ble_bridge/README.md).

### 7.1. Bảng đầy đủ lệnh Bluetooth (BLE) / UART gửi tới STM32

| Nhóm | Lệnh (không phân biệt hoa/thường) | Chức năng & Ví dụ |
| :--- | :--- | :--- |
| **Điều khiển lái** | `F` / `FORWARD` | Chạy tiến êm (`0.15 m/s`, có ramp mượt; tự dừng sau 500 ms nếu nhả nút) |
| | `B` / `BACKWARD` | Chạy lùi êm (`-0.15 m/s`, có ramp mượt) |
| | `L` / `LEFT` | Quay tại chỗ sang trái (`turn_offset = -150`) |
| | `R` / `RIGHT` | Quay tại chỗ sang phải (`turn_offset = +150`) |
| | `S` / `HALT` | Dừng di chuyển, giữ thăng bằng đứng yên tại chỗ |
| **Chế độ vận hành** | `AUTO` / `START` / `RUN` | Bật chế độ tự cân bằng (dựng xe trong `±10°` đủ `0.2 s` sẽ chạy) |
| | `OFF` / `STOP` | Ngắt khẩn cấp động cơ (`STBY = LOW`, `PWM = 0`) |
| **Tinh chỉnh nóng (RAM)** | `SET_PID:kp,ki,kd,skp,ski` | Đổi 5 hệ số PID ngay lập tức. VD: `SET_PID:40,0,1.4,28,2.5` (hoặc `PID:...`) |
| | `GET_PID` / `PID?` | Đọc 5 hệ số PID hiện tại |
| | `OFFSET:<góc>` / `SET_OFFSET:<góc>` | Chỉnh góc cân bằng cơ học (độ). VD: `OFFSET:-3.60` |
| | `GET_OFFSET` / `OFFSET?` | Xem góc Offset hiện tại |
| | `DEADZONE:<val>` / `SET_DEADZONE:<val>` | Dịch vùng chết 2 bánh (`0..400`, giữ nguyên độ lệch Trái/Phải; `0` = tắt) |
| | `DEADZONE?` / `GET_DEADZONE` | Xem giá trị Deadzone tham chiếu hiện tại |
| | `SET_POLARITY:ml,mr,el,er` | Đảo chiều cực tính motor/encoder (`+1` hoặc `-1`). VD: `SET_POLARITY:1,-1,-1,1` |
| **Tra cứu & Tiện ích** | `PARAMS` / `INFO` / `STATUS` | In bảng tổng hợp toàn bộ PID, Offset, Deadzone, chế độ, Pitch, Gyro, vận tốc, Encoder |
| | `HELP` / `?` | In nhanh danh sách lệnh trên điện thoại |
| | `PING` / `ESP:PING` | Kiểm tra thông tuyến (`PONG` / `ESP:PONG`) |
| | `RESET_ENC` / `ZERO_ENC` | Đặt lại bộ đếm xung 2 encoder về `(0, 0)` |
| | `START_STREAM` / `STOP_STREAM` | Bật / tắt phát luồng telemetry 100 Hz (`DATA:` + `DBG:`) |
| **Kiểm thử phần cứng** | `PROBE` / `PROBE:A` / `PROBE:B` / `PROBE:PWM` | Khóa cứng `STBY = 0V` và kích `AIN/BIN/PWM` lên `3.3V` để đo đồng hồ VOM an toàn |
| | `MOTOR:<pwm_l>,<pwm_r>` | Chạy trực tiếp 2 bánh qua bù Deadzone & Scale. VD: `MOTOR:200,200`, `MOTOR:0,0` |
| | `TEST:MOTOR_L` / `TEST:MOTOR_R` / `TEST:MOTORS` | Chạy thử nhanh từng bánh hoặc cả 2 bánh trong `0.6–0.8 s` rồi tự tắt |
| | `LED:ON` / `LED:OFF` / `LED:TOGGLE` / `LED:AUTO` | Kiểm tra thủ công đèn LED `PC13` trên BlackPill |

### 7.2. Định dạng luồng Telemetry STM32 phát lên (khi bật `START_STREAM`)
Mỗi chu kỳ `10 ms` (100 Hz), STM32 phát 2 dòng ASCII đồng bộ:
```text
DATA:<pitch_deg>,<gyro_dps>,<pwm_avg>,<enc_left>,<enc_right>,<fsm_state>
DBG:<target_angle_deg>,<speed_filtered_mps>,<speed_i_deg>,<pwm_left>,<pwm_right>
```
*Ví dụ:*
```text
DATA:-3.55,1.20,14,120,118,1
DBG:-0.18,0.025,-0.04,14,-13
```
* `fsm_state`: `0` = `IDLE`, `1` = `BALANCING`, `2` = `RECOVERY`.
* `pwm_left`, `pwm_right`: lệnh PWM đã nhân `MOTOR_SCALE` và cực tính chiều quay từng bánh.

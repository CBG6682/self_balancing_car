# Hệ Thống Xe Hai Bánh Tự Cân Bằng (STM32F401CCU6 Black Pill)
## Dual-Loop Cascade PID Control, 1D Kalman Filter, Robot FSM & Automated Python Tuning Pipeline

[![STM32](https://img.shields.io/badge/MCU-STM32F401CCU6-002B49?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f401cc.html)
[![Toolchain](https://img.shields.io/badge/Toolchain-Arm_GNU_GCC_%2B_CMake-blue?logo=cmake)](https://cmake.org/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Dự án phát triển firmware nhúng hoàn chỉnh và chuỗi công cụ tự động hóa toán học cho **Xe Hai Bánh Tự Cân Bằng (Two-Wheeled Self-Balancing Robot)**. Hệ thống kết hợp kiến trúc điều khiển thời gian thực trên vi điều khiển STM32F401CCU6 (ARM Cortex-M4 @ 84MHz) với pipeline mô phỏng vật lý phi tuyến, nhận dạng hệ thống ARX và tối ưu hóa PID tự động (Nelder-Mead ITAE) bằng Python.

---

## MỤC LỤC
1. [Tính Năng Nổi Bật](#1-tính-năng-nổi-bật)
2. [Sơ Đồ Đấu Nối Phần Cứng (Hardware Pinout)](#2-sơ-đồ-đấu-nối-phần-cứng-hardware-pinout)
3. [Kiến Trúc Phần Mềm (Software Architecture)](#3-kiến-trúc-phần-mềm-software-architecture)
4. [Cấu Trúc Thư Mục Dự Án](#4-cấu-trúc-thư-mục-dự-án)
5. [Yêu Cầu Môi Trường & Cài Đặt](#5-yêu-cầu-môi-trường--cài-đặt)
6. [Biên Dịch & Nạp Firmware](#6-biên-dịch--nạp-firmware)
7. [Chuỗi Công Cụ Tự Động Hóa & Tối Ưu PID](#7-chuỗi-công-cụ-tự-động-hóa--tối-ưu-pid)
8. [Quy Trình Tinh Chỉnh Khi Thay Đổi Khung Xe](#8-quy-trình-tinh-chỉnh-khi-thay-đổi-khung-xe)
9. [Đặc Tả Giao Thức Telemetry UART](#9-đặc-tả-giao-thức-telemetry-uart)

---

## 1. TÍNH NĂNG

- **Thuật toán Cân bằng Đa vòng (Cascade PID Dual-Loop)**:
  - **Vòng trong (Pitch Angle PD/PID @ 100Hz)**: Ổn định góc nghiêng cực nhanh. Sử dụng trực tiếp tốc độ quay đo từ con quay hồi chuyển (`gyro_rate`) cho khâu vi phân, loại bỏ hoàn toàn hiện tượng xung giật đạo hàm (*derivative kick*) do sai số đo tức thời.
  - **Vòng ngoài (Linear Speed PI @ 100Hz)**: Đọc vận tốc thực từ encoder, qua bộ lọc số thông thấp IIR ($0.75 / 0.25$) để triệt tiêu nhiễu lượng tử hóa xung, sinh góc nghiêng mục tiêu $\theta_{target} \in [-10^\circ, +10^\circ]$.
  - **Anti-Windup Clamping**: Giới hạn tích phân chống bão hòa cơ cấu chấp hành.
- **Xử lý Cảm biến IMU Chuyên sâu**:
  - Giao tiếp MPU6050 qua I2C1 Fast Mode (400 kHz), lọc số phần cứng DLPF 42Hz.
  - Tự động đo 300 mẫu lúc khởi động để triệt tiêu trôi lệch tĩnh (*gyro zero-rate bias*).
  - Thuật toán lọc Kalman 1D tối ưu hóa tốc độ thực thi, dung hòa gia tốc kế trọng trường và con quay hồi chuyển.
- **Cầu H TB6612FNG & Encoder 2 Bánh**:
  - Phát PWM 20kHz không phát ra tiếng rít âm tần bằng TIM4 CH3/CH4 (ARR = 4199).
  - Đọc xung góc 4x Encoder đồng thời bằng TIM2 (Bánh trái) và TIM3 (Bánh phải), tự động xử lý tràn số 16-bit.
  - Tuyến tính hóa và bù vùng chết động cơ GA25 (`MOTOR_PWM_DEADZONE = 450`) giúp xe phản ứng tức thì ở dải vận tốc siêu chậm.
- **Máy Trạng Thái Hữu Hạn Robot (FSM)**:
  - `STATE_BALANCING`: Giữ thăng bằng và nhận lệnh điều khiển di chuyển. Tự động phanh hãm đứng yên khi ngừng phát lệnh.
  - `STATE_RECOVERY`: Tự động phát xung lực hồi phục cực đại ($\pm 3800$ PWM) khi góc nghiêng trong khoảng $20^\circ < |\theta| \le 45^\circ$. Timeout 1.5s bảo vệ động cơ.
  - `STATE_IDLE`: Ngắt cầu H (kéo chân PB0 `STBY` xuống LOW và PWM = 0) khi xe ngã quá $45^\circ$, bảo vệ cháy motor và quá dòng pin.
- **Giao Tiếp Không Chặn & Nạp Nóng PID**:
  - USART1 Ring Buffer 256 bytes với ngắt Rx, không làm chậm vòng quét 10ms.
  - Hỗ trợ nạp nóng hệ số PID trực tiếp vào RAM qua lệnh `SET_PID:...` mà không cần flash lại chip.
- **Pipeline Tự Động Hóa Toán Học (Python Toolchain)**:
  - Tính nghiệm LQR giải Algebraic Riccati Equation (CARE) có cơ chế MD5 Cache.
  - Thu thập dữ liệu thực tế kèm cơ chế Fallback mô phỏng phi tuyến Runge-Kutta 4 (RK4).
  - Nhận dạng hệ thống hàm truyền ARX(2,2) Least Squares đạt $R^2 > 96\%$.
  - Tối ưu hóa hệ số PID bằng thuật toán Nelder-Mead (ITAE) đưa sai số xác lập về $< 1.0^\circ$.
  - Kiểm thử kịch bản tự động (`step6_test_control_sequence.py`) đánh giá tự động PASS/FAIL và xuất đồ thị phản hồi 3 kênh.

---

## 2. SƠ ĐỒ ĐẤU NỐI PHẦN CỨNG (HARDWARE PINOUT)

Toàn bộ cấu hình chân khớp 100% với mã nguồn CubeMX (`Core/Src/gpio.c`, `tim.c`, `i2c.c`, `usart.c`):

| Phân Hệ | Chân Module | Chân STM32 | Tên Cổng & Ngoại Vi | Chức Năng Chi Tiết | Điện Áp |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **MPU6050** | VCC | 3.3V / 5V | 3.3V Pin | Cấp nguồn cảm biến IMU | 3.3V |
| | GND | GND | GND Pin | Mass chung toàn hệ thống | 0V |
| | SCL | **PB6** | I2C1_SCL | Xung nhịp I2C 400kHz | 3.3V |
| | SDA | **PB7** | I2C1_SDA | Đường dữ liệu I2C | 3.3V |
| **TB6612FNG** | VM | LiPo (+) | Nguồn pin ngoài | Nguồn cấp động cơ GA25 | 11.1V - 12.6V |
| | VCC | 3.3V / 5V | 3.3V Pin | Cấp nguồn mạch logic TB6612 | 3.3V |
| | GND | GND | GND Pin | Mass chung nối với Pin & MCU | 0V |
| | STBY | **PB0** | GPIO_Output | Bật/Tắt cầu H (HIGH=Chạy, LOW=Ngắt) | 3.3V |
| | PWMA | **PB8** | TIM4_CH3 | Tốc độ bánh trái (PWM 20kHz, ARR=4199) | 3.3V Logic |
| | PWMB | **PB9** | TIM4_CH4 | Tốc độ bánh phải (PWM 20kHz, ARR=4199) | 3.3V Logic |
| | AIN1 | **PA4** | GPIO_Output | Chiều quay motor trái (Kênh A1) | 3.3V Logic |
| | AIN2 | **PA5** | GPIO_Output | Chiều quay motor trái (Kênh A2) | 3.3V Logic |
| | BIN1 | **PA6** | GPIO_Output | Chiều quay motor phải (Kênh B1) | 3.3V Logic |
| | BIN2 | **PA7** | GPIO_Output | Chiều quay motor phải (Kênh B2) | 3.3V Logic |
| | AO1, AO2 | Motor Trái | - | Ngõ ra động cơ GA25 trái | Tới 12V PWM |
| | BO1, BO2 | Motor Phải | - | Ngõ ra động cơ GA25 phải | Tới 12V PWM |
| **Encoder Trái**| VCC / GND | 3.3V / GND | - | Cấp nguồn Hall sensor | 3.3V |
| | Phase A | **PA0** | TIM2_CH1 | Kênh A Encoder bánh trái (Mode TI12) | 3.3V |
| | Phase B | **PA1** | TIM2_CH2 | Kênh B Encoder bánh trái (Mode TI12) | 3.3V |
| **Encoder Phải**| VCC / GND | 3.3V / GND | - | Cấp nguồn Hall sensor | 3.3V |
| | Phase A | **PB4** | TIM3_CH1 | Kênh A Encoder bánh phải (Mode TI12) | 3.3V |
| | Phase B | **PB5** | TIM3_CH2 | Kênh B Encoder bánh phải (Mode TI12) | 3.3V |
| **Bluetooth / UART**| VCC / GND | 5V / GND | 5V / GND Pin | Cấp nguồn HC-05 hoặc cáp USB-UART | 5V / 3.3V |
| | TXD (Module) | **PA10** | USART1_RX | Nhận lệnh điều khiển / tuning | 3.3V |
| | RXD (Module) | **PA9** | USART1_TX | Phát dữ liệu Telemetry tốc độ cao | 3.3V |
| **Định Thời Chu Kỳ** | Nội bộ | - | **TIM10** | Ngắt chu kỳ điều khiển 10ms (100Hz) | Nội bộ |

> [!WARNING]
> Cực âm (`GND`) của Pin LiPo 3S, vi điều khiển STM32 và mạch cầu H TB6612 **BẮT BUỘC PHẢI NỐI CHUNG** với nhau để đảm bảo mức điện áp tham chiếu đồng nhất.

---

## 3. KIẾN TRÚC PHẦN MỀM (SOFTWARE ARCHITECTURE)

```
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
|  & Kalman  |-->|  Dual-Loop Core   |------------->|  & 4x Encoders   |   |    UART    |
| (PB6/PB7)  |   | (Inner PD+Out PI) |   (PWM L/R)  | (PB8/9, PA0/1/4) |   | (PA9/PA10) |
+------------+   +---------+---------+              +------------------+   +-----+------+
                           |                                                     ^
                           v                                                     |
                 +-------------------+                                           |
                 |     Robot FSM     |-------------------------------------------+
                 | (BALANCING, RECOV)|          (Telemetry Status Broadcast)
                 +-------------------+
```

---

## 4. CẤU TRÚC THƯ MỤC DỰ ÁN

```
self_balancing_car/
├── CMakeLists.txt              # Cấu hình build CMake đa nền tảng
├── CMakePresets.json           # Cấu hình Presets Debug / Release
├── setup.sh / setup.bat        # Script tự động cài môi trường Python venv
├── build_firmware.sh / .bat    # Script biên dịch firmware ra .elf, .bin, .hex
├── run_pipeline.sh / .bat      # Script chạy chuỗi tối ưu hóa PID tự động
├── .gitignore                  # Cấu hình Git tiêu chuẩn
├── Core/
│   ├── Inc/                    # Header cấu hình CubeMX (main.h, tim.h, ...)
│   └── Src/                    # Mã nguồn chính CubeMX (main.c, gpio.c, ...)
├── Modules/                    # Thư viện điều khiển xe tự cân bằng chuyên dụng
│   ├── Inc/
│   │   ├── pid_AutoConfig.h    # File tham số PID tự động sinh bởi Python
│   │   ├── cascade_pid.h       # Khai báo cấu trúc & hàm Cascade PID
│   │   ├── mpu6050_filter.h    # Khai báo driver MPU6050 & Lọc Kalman
│   │   ├── motor_encoder.h     # Khai báo driver TB6612 & Encoder 4x
│   │   ├── robot_fsm.h         # Khai báo máy trạng thái hữu hạn FSM
│   │   └── telemetry_uart.h    # Khai báo ring buffer UART & parser lệnh
│   └── Src/
│       ├── cascade_pid.c       # Hiện thực vòng lặp PID kép & chống bão hòa
│       ├── mpu6050_filter.c    # Hiện thực giao tiếp I2C1 & Kalman 1D
│       ├── motor_encoder.c     # Hiện thực điều khiển động cơ & bù deadzone
│       ├── robot_fsm.c         # Hiện thực logic chuyển trạng thái FSM
│       └── telemetry_uart.c    # Hiện thực ngắt nhận USART1 & Telemetry
└── Tools/                      # Bộ công cụ mô phỏng & tối ưu hóa PID (Python)
    ├── requirements.txt        # Danh sách thư viện Python cần thiết
    ├── robot_config.yaml       # File cấu hình thông số vật lý của khung xe
    ├── computed_gains.json     # Cache lưu trữ bộ thông số PID tối ưu
    ├── mock_telemetry.csv      # File dữ liệu mô phỏng Runge-Kutta 4
    ├── test_result_plot.png    # Đồ thị kiểm thử kịch bản tự động
    ├── step2_sim_and_init_pid.py      # B2: Giải LQR & khởi tạo PID có cache
    ├── step4_data_acquisition.py      # B4: Thu thập UART hoặc sinh Mock RK4
    ├── step5_auto_tune_pid.py         # B5: Nhận dạng ARX & Nelder-Mead ITAE
    └── step6_test_control_sequence.py # B6: Kiểm thử kịch bản & xuất đồ thị
```

---

## 5. YÊU CẦU MÔI TRƯỜNG & CÀI ĐẶT

### Trên Linux (Ubuntu / Debian / Arch):
```bash
# Cài đặt Arm GCC Toolchain, CMake và Ninja
sudo apt update
sudo apt install -y gcc-arm-none-eabi cmake ninja-build python3 python3-venv python3-pip

# Khởi tạo môi trường ảo Python và cài đặt dependencies
./setup.sh
```

### Trên Windows (Command Prompt / PowerShell):
1. Tải và cài đặt **Arm GNU Toolchain** (chọn tùy chọn thêm vào `PATH`).
2. Tải và cài đặt **CMake** (từ cmake.org) và **Python 3.8+** (chọn tick *"Add python.exe to PATH"*).
3. Chạy file cài đặt môi trường:
```cmd
setup.bat
```

---

## 6. BIÊN DỊCH & NẠP FIRMWARE

### Biên dịch:
- **Linux / macOS**:
  ```bash
  ./build_firmware.sh
  ```
- **Windows**:
  ```cmd
  build_firmware.bat
  ```

File đầu ra được lưu tại `build/Debug/`:
- `self_balancing_car.elf`: Chứa bảng biểu tượng phục vụ gỡ lỗi SWD/JTAG.
- `self_balancing_car.bin`: Mã máy nhị phân thuần túy nạp trực tiếp vào Flash.
- `self_balancing_car.hex`: Định dạng Intel Hex.

### Nạp Firmware:
Sử dụng mạch nạp **ST-Link v2** nối với chân `SWDIO` (PA13), `SWCLK` (PA14), `GND`, `3.3V`:
- **Qua OpenOCD / st-flash (Linux)**:
  ```bash
  st-flash write build/Debug/self_balancing_car.bin 0x08000000
  ```
- **Qua STM32CubeProgrammer**:
  Mở phần mềm, kết nối ST-Link, chọn file `build/Debug/self_balancing_car.hex` và nhấn **Start Programming**.

---

## 7. CHUỖI TỰ ĐỘNG HÓA & TỐI ƯU PID

Dự án cung cấp bộ công cụ tự động hóa khép kín không phụ thuộc vào việc phải căn chỉnh thủ công từng hệ số PID:

### Chạy trọn vẹn quy trình (khuyến nghị):
- **Linux**: `./run_pipeline.sh --all`
- **Windows**: `run_pipeline.bat --all`

Quy trình sẽ tự động thực hiện 4 bước liên hoàn:
1. **Step 2 (LQR Solver & Caching)**: Đọc mô hình vật lý từ `Tools/robot_config.yaml`, giải phương trình Riccati liên tục, phân bổ nghiệm LQR thành hệ số Cascade PID và lưu vào `Tools/computed_gains.json`.
2. **Step 4 (Data Acquisition)**: Tự động quét các cổng serial gửi bắt tay `PING`. Nếu chưa kết nối robot thật, tự động kích hoạt bộ sinh dữ liệu vật lý phi tuyến Runge-Kutta 4 xuất ra file `Tools/mock_telemetry.csv`.
3. **Step 5 (System Identification & Nelder-Mead Optimization)**: Khớp mô hình hàm truyền rời rạc ARX(2,2) bằng thuật toán Bình phương Tối thiểu (Least Squares) đạt $R^2 > 96\%$, sau đó tối ưu hóa hệ số PID giảm thiểu hàm phạt ITAE. Tự động ghi đè file header C `Modules/Inc/pid_AutoConfig.h`.
4. **Step 6 (Control Sequence Testing)**: Chạy kịch bản tự động gồm 6 giai đoạn (`BALANCE` $\rightarrow$ `FORWARD` $\rightarrow$ `STOP` $\rightarrow$ `BACKWARD` $\rightarrow$ `TURN` $\rightarrow$ `STOP`), tự động chấm điểm và lưu ảnh đồ thị `Tools/test_result_plot.png`.
5. **Auto-Rebuild**: Tự động gọi biên dịch lại firmware STM32 với bộ thông số mới.

---

## 8. QUY TRÌNH TINH CHỈNH KHI THAY ĐỔI KHUNG XE

Khi bạn chế tạo khung xe mới, thay pin có trọng lượng khác hoặc đổi kích thước bánh xe, bạn **KHÔNG CẦN** sửa bất kỳ dòng mã nguồn C nào. Hãy làm theo 3 bước sau:

1. Mở file [Tools/robot_config.yaml](Tools/robot_config.yaml) và cập nhật thông số thực tế của bạn:
   ```yaml
   chassis:
     M: 0.85          # Khối lượng thân xe (kg)
     m: 0.08          # Khối lượng 2 bánh xe (kg)
     R: 0.0325        # Bán kính bánh xe (m) (Ví dụ: bánh D=65mm -> R=0.0325)
     L: 0.075         # Khoảng cách từ trục bánh xe tới khối tâm COM (m)
     W: 0.16          # Khoảng cách giữa 2 bánh xe (m)
     I_body: 0.0035   # Mô men quán tính thân xe (kg*m^2)
   motor:
     pwm_max: 4199
     pwm_deadzone: 450
     encoder_cpr: 330 # Số xung encoder sau khi qua hộp số
   ```
2. Chạy lệnh tự động hóa:
   ```bash
   ./run_pipeline.sh --all
   ```
3. Nạp file `build/Debug/self_balancing_car.bin` mới sinh vào STM32. Robot sẽ tự động đạt trạng thái cân bằng tối ưu!

---

## 9. ĐẶC TẢ GIAO THỨC TELEMETRY UART

Tốc độ Baud mặc định: **115200 bps**, 8 data bits, no parity, 1 stop bit (USART1 - PA9/PA10).

### Lệnh gửi từ Máy tính / App Bluetooth tới STM32:
- `PING\n`: Bắt tay kết nối. STM32 sẽ phản hồi `PONG\n`.
- `START_STREAM\n`: Bật chế độ phát dữ liệu Telemetry định kỳ 10ms.
- `STOP_STREAM\n`: Dừng phát dữ liệu Telemetry.
- `SET_PID:<ang_kp>,<ang_ki>,<ang_kd>,<spd_kp>,<spd_ki>\n`: Cập nhật trực tiếp bộ thông số PID vào RAM khi xe đang đứng. Ví dụ: `SET_PID:120.0,1.5,3.2,8.0,0.5\n`. Phản hồi: `ACK:PID_UPDATED\n`.
- `FORWARD\n`, `BACKWARD\n`, `LEFT\n`, `RIGHT\n`, `STOP\n`: Các lệnh điều khiển chuyển động thủ công.

### Dữ liệu Telemetry STM32 phát lên (khi bật stream):
Định dạng chuỗi ASCII kết thúc bằng ký tự xuống dòng `\n`:
```
DATA:<pitch_angle>,<gyro_rate>,<pwm_average>,<enc_left_pulses>,<enc_right_pulses>,<fsm_state>\n
```
*Ví dụ*: `DATA:1.24,-2.50,150,1204,1198,1\n`
- `pitch_angle`: Góc nghiêng Pitch hiện tại (độ).
- `gyro_rate`: Tốc độ quay quanh trục Pitch (độ/giây).
- `pwm_average`: Giá trị PWM trung bình cấp cho 2 motor (-4199 đến 4199).
- `enc_left_pulses`, `enc_right_pulses`: Tổng số xung encoder tích lũy.
- `fsm_state`: Trạng thái máy hữu hạn (1: BALANCING, 2: RECOVERY, 0: IDLE).

---

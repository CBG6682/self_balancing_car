# HƯỚNG DẪN KIỂM THỬ TỪNG LINH KIỆN & HIỆU CHUẨN THAM SỐ
## Dự Án Xe Tự Cân Bằng STM32F401CCU6 + ESP32-S3 BLE

Tài liệu này mô tả quy trình kiểm thử **từ dưới lên (Bottom-Up)** cho từng phần cứng và các bước hiệu chuẩn tham số (`Deadzone`, `Offset`, `PID`, `sKp/sKi`) bằng bộ công cụ Python hiện có trong thư mục `Tools/`.

---

## MỤC LỤC
1. [Sơ Đồ Đấu Dây Chuẩn Toàn Hệ Thống](#1-sơ-đồ-đấu-dây-chuẩn-toàn-hệ-thống)
2. [Bước 1: Kiểm Tra Kết Nối USB-UART & Cầu Nối ESP32-S3](#2-bước-1-kiểm-tra-kết-nối-usb-uart--cầu-nối-esp32-s3)
3. [Bước 2: Kiểm Tra Tổng Thể Phần Cứng (`system_self_test.py`)](#3-bước-2-kiểm-tra-tổng-thể-phần-cứng-system_self_testpy)
4. [Bước 3: Kiểm Tra Chân Cầu H TB6612 Bằng Đồng Hồ VOM (`PROBE`)](#4-bước-3-kiểm-tra-chân-cầu-h-tb6612-bằng-đồng-hồ-vom-probe)
5. [Bước 4: Hiệu Chuẩn Cảm Biến MPU6050 & Encoder 4x (`live_sensor_calib.py`)](#5-bước-4-hiệu-chuẩn-cảm-biến-mpu6050--encoder-4x-live_sensor_calibpy)
6. [Bước 5: Đo Deadzone Động Cơ & Tìm Góc Offset Cơ Khí](#6-bước-5-đo-deadzone-động-cơ--tìm-góc-offset-cơ-khí)
7. [Bước 6: Tinh Chỉnh PID, Giám Sát & Ghi Telemetry 100 Hz](#7-bước-6-tinh-chỉnh-pid-giám-sát--ghi-telemetry-100-hz)
8. [Bảng Lệnh Chạy Nhanh Đa Nền Tảng (Linux / Windows)](#8-bảng-lệnh-chạy-nhanh-đa-nền-tảng-linux--windows)

---

## 1. SƠ ĐỒ ĐẤU DÂY CHUẨN TOÀN HỆ THỐNG

### 1.1. Cầu H TB6612FNG ↔ STM32F401CCU6
| Chân TB6612FNG | Chân STM32 | Chức năng |
| :--- | :---: | :--- |
| **STBY** | **PA2** | Bật/Tắt cầu H (`HIGH` = Chạy, `LOW` = Ngắt an toàn) |
| **PWMA** | **PB8** | `TIM4_CH3` — PWM bánh trái (`21 kHz`, `ARR = 999`, trần 80% = `800`) |
| **AIN1** | **PA4** | Chiều động cơ trái A1 |
| **AIN2** | **PA5** | Chiều động cơ trái A2 |
| **PWMB** | **PB9** | `TIM4_CH4` — PWM bánh phải (`21 kHz`, `ARR = 999`, trần 80% = `800`) |
| **BIN1** | **PA3** | Chiều động cơ phải B1 |
| **BIN2** | **PA7** | Chiều động cơ phải B2 |
| **VM / VCC / GND** | LiPo 3S / 3.3V / GND | Nguồn công suất 11.1–12.6V, nguồn logic 3.3V, nối chung Mass |

### 1.2. Cảm Biến MPU6050 ↔ STM32F401CCU6
| Chân MPU6050 | Chân STM32 | Chức năng |
| :--- | :---: | :--- |
| **SCL** | **PB6** | `I2C1_SCL` (400 kHz, địa chỉ `0x68`) |
| **SDA** | **PB7** | `I2C1_SDA` |
| **VCC / GND** | 3.3V / GND | Nguồn cảm biến 3.3V |

### 1.3. Encoder GA25 (21.3:1 — 937 CPR) ↔ STM32F401CCU6
| Chân Encoder | Chân STM32 | Chức năng |
| :--- | :---: | :--- |
| **Trái — Phase A / B** | **PA0 / PA1** | `TIM2_CH1 / CH2` (4x Encoder Mode) |
| **Phải — Phase A / B** | **PB4 / PB5** | `TIM3_CH1 / CH2` (4x Encoder Mode) |

### 1.4. ESP32-S3 BLE Bridge ↔ STM32F401CCU6
| Chân ESP32-S3 | Chân STM32 | Chức năng |
| :--- | :---: | :--- |
| **GPIO 6 (TX1)** | **PA10** (`USART1_RX`) | Lệnh từ App BLE / PC → STM32 (`115200 bps`) |
| **GPIO 3 (RX1)** | **PA9** (`USART1_TX`) | Telemetry từ STM32 → App BLE / PC (`115200 bps`) |
| **5V / GND** | 5V Buck / GND | Cấp nguồn 5V riêng (≥ 500 mA), nối chung GND |

---

## 2. BƯỚC 1: KIỂM TRA KẾT NỐI USB-UART & CẦU NỐI ESP32-S3

* **Công cụ:** [`Tools/check_connection.py`](check_connection.py)
* **Lệnh chạy:**
  ```bash
  ./run_pipeline.sh -c
  # hoặc: ./venv/bin/python3 Tools/check_connection.py
  ```
* **Tiêu chí đạt:** Tự phát hiện cổng `/dev/ttyACM0` (hoặc `COMx` trên Windows), gửi `PING` nhận `PONG` và gửi `PARAMS` hiển thị đầy đủ thông số hiện tại của STM32.

---

## 3. BƯỚC 2: KIỂM TRA TỔNG THỂ PHẦN CỨNG (`system_self_test.py`)

* **Công cụ:** [`Tools/system_self_test.py`](system_self_test.py)
* **Lệnh chạy:**
  ```bash
  ./run_pipeline.sh -d
  ```
* **Chu trình kiểm thử tuần tự 7 bước có xác nhận người dùng:**
  1. Phát hiện cổng & bắt tay ESP32-S3 / STM32 (`PING` / `STATUS`).
  2. Kiểm tra cảm biến IMU MPU6050 (`WHO_AM_I = 0x68`, góc Pitch & Gyro).
  3. Kiểm tra xung Encoder 4x bánh trái (`TIM2`) & bánh phải (`TIM3`).
  4. Kiểm tra cầu H TB6612FNG & động cơ Trái (tiến/lùi).
  5. Kiểm tra cầu H TB6612FNG & động cơ Phải (tiến/lùi).
  6. Đồng bộ 2 bánh & xác nhận ngắt an toàn (`OFF` / `STBY = LOW`).

> [!IMPORTANT]
> Đặt xe **kê nổi 2 bánh lên khỏi mặt bàn** trước khi chạy kiểm tra động cơ.

---

## 4. BƯỚC 3: KIỂM TRA CHÂN CẦU H TB6612 BẰNG ĐỒNG HỒ VOM (`PROBE`)

Khi nghi ngờ lỏng dây điều khiển chiều (ví dụ `BIN2 / PA7` chỉ mất 1 chiều quay) hoặc đứt dây PWM, sử dụng chế độ **Hardware Probe an toàn** (STM32 khóa cứng `STBY = PA2 = 0.0V` nên động cơ tuyệt đối không quay):

| Lệnh gửi qua BLE / Terminal | Điện áp chuẩn khi đo bằng VOM tại chân TB6612 |
| :--- | :--- |
| **`PROBE`** (hoặc `PROBE:ALL`) | `STBY = 0.0V`; `AIN1 (PA4)`, `AIN2 (PA5)`, `BIN1 (PA3)`, `BIN2 (PA7)`, `PWMA (PB8)`, `PWMB (PB9)` = **`3.3V`** |
| **`PROBE:A`** | `STBY = 0.0V`; `AIN1`, `AIN2`, `PWMA` = **`3.3V`**; kênh B = `0.0V` |
| **`PROBE:B`** | `STBY = 0.0V`; `BIN1`, `BIN2`, `PWMB` = **`3.3V`**; kênh A = `0.0V` |
| **`PROBE:PWM`** | `STBY = 0.0V`; `PWMA`, `PWMB` = **`3.3V`**; `AIN/BIN` = `0.0V` |
| **`OFF`** / **`AUTO`** | Ngắt toàn bộ về `0.0V` / quay lại chế độ tự cân bằng |

---

## 5. BƯỚC 4: HIỆU CHUẨN CẢM BIẾN MPU6050 & ENCODER 4X (`live_sensor_calib.py`)

* **Lệnh chạy:** `./run_pipeline.sh -s`
* **Xác nhận bắt buộc:**
  1. Nghiêng đầu xe về **phía trước**: `Pitch` tăng theo chiều dương, `Gyro` dương.
  2. Lăn cả 2 bánh về **phía trước**: `enc_left` và `enc_right` cùng tăng dương; quay đúng 1 vòng bánh xe đếm xấp xỉ **`937` xung** (`ENCODER_CPR = 937`).
  3. Nếu ngược chiều, chỉnh cực tính trong `Modules/Inc/pid_AutoConfig.h` (hoặc thử nóng bằng `SET_POLARITY:ml,mr,el,er`).

---

## 6. BƯỚC 5: ĐO DEADZONE ĐỘNG CƠ & TÌM GÓC OFFSET CƠ KHÍ

1. **Đo Deadzone từng bánh (`find_deadzone.py`):**
   ```bash
   ./run_pipeline.sh -dz
   ```
   Tăng dần PWM từ `0` để tìm ngưỡng bắt đầu nhúc nhích của từng động cơ ở trạng thái có tải (xe đặt trên sàn). Chọn giá trị bù bằng **~85%** ngưỡng đo được để không gây giật quá đà (`PWM_DEADZONE_LEFT = 23`, `PWM_DEADZONE_RIGHT = 17`), đồng thời dùng `MOTOR:300,300` so sánh xung encoder 2 bánh để chốt hệ số cân bằng `MOTOR_RIGHT_SCALE = 0.93`.

2. **Tìm góc Offset cơ khí (`find_zero_balance.py`):**
   ```bash
   ./run_pipeline.sh -o
   ```
   Giữ xe đứng cân bằng tự nhiên tại tâm trọng lực, lấy góc Pitch trung bình làm điểm khởi đầu, sau đó vi chỉnh qua Bluetooth (`OFFSET:-3.60`) tới khi chạm nhẹ xe không còn ưu tiên đổ về 1 hướng cố định.

---

## 7. BƯỚC 6: TINH CHỈNH PID, GIÁM SÁT & GHI TELEMETRY 100 HZ

* **Giám sát & tinh chỉnh nóng qua ESP32 Bridge:**
  ```bash
  ./run_pipeline.sh -m     # Tools/esp32_tune_monitor.py
  ./run_pipeline.sh -t     # Tools/live_balance_tuning.py
  ```
* **Ghi phiên dữ liệu 100 Hz (`DATA:` + `DBG:`) để phân tích dao động:**
  ```bash
  ./run_pipeline.sh -r     # Tools/record_session.py
  ```
* **Mô phỏng LQR & tối ưu PID từ `robot_config.yaml`:**
  ```bash
  ./run_pipeline.sh --sim       # Tools/sim_and_init_pid.py
  ./run_pipeline.sh --autotune  # Tools/auto_tune_pid.py
  ```

---

## 8. BẢNG LỆNH CHẠY NHANH ĐA NỀN TẢNG (LINUX / WINDOWS)

| Tác vụ | Linux / macOS | Windows |
| :--- | :--- | :--- |
| Mở menu tổng hợp | `./run_pipeline.sh` | `run_pipeline.bat` |
| Kiểm tra tổng thể linh kiện | `./run_pipeline.sh -d` | `run_pipeline.bat -d` |
| Kiểm tra kết nối UART/ESP32 | `./run_pipeline.sh -c` | `run_pipeline.bat -c` |
| Hiệu chuẩn MPU6050 & Encoder | `./run_pipeline.sh -s` | `run_pipeline.bat -s` |
| Đo Deadzone động cơ | `./run_pipeline.sh -dz` | `run_pipeline.bat -dz` |
| Tìm góc Offset cân bằng | `./run_pipeline.sh -o` | `run_pipeline.bat -o` |
| Biên dịch firmware STM32 | `./build_firmware.sh` | `build_firmware.bat` |
| Nạp firmware qua ST-Link | `./Tools/flash_stlink.sh` | `Tools\flash_stlink.bat` |

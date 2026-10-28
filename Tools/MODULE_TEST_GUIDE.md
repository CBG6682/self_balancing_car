# HƯỚNG DẪN KIỂM TRA ĐỘC LẬP TỪNG MODULE PHẦN CỨNG
## DỰ ÁN XE TỰ CÂN BẰNG STM32F401CCU6 + ESP32-S3 BLE

Tài liệu này cung cấp quy trình và bộ công cụ độc lập để kiểm tra từng module phần cứng (Hardware Bringup & Unit Testing) trước khi tiến hành chạy thuật toán tự cân bằng.

---

## MỤC LỤC
1. [Kiến Trúc & Phân Vai Phần Cứng](#1-kiến-trúc--phân-vai-phần-cứng)
2. [Sơ Đồ Đấu Dây Chuẩn Toàn Hệ Thống](#2-sơ-đồ-đấu-dây-chuẩn-toàn-hệ-thống)
3. [Module 1: Kiểm Tra Mạch Cầu Nối ESP32-S3 (`test_esp32_loopback.py`)](#3-module-1-kiểm-tra-mạch-cầu-nối-esp32-s3-test_esp32_loopbackpy)
4. [Module 2: Kiểm Tra Thông Tuyến UART Vật Lý (`test_bridge_live.py`)](#4-module-2-kiểm-tra-thông-tuyến-uart-vật-lý-test_bridge_livepy)
5. [Module 3: Kiểm Tra Động Cơ & Mạch Cầu H TB6612 (`motor_test_interactive.py` & Auto Test)](#5-module-3-kiểm-tra-động-cơ--mạch-cầu-h-tb6612)
6. [Module 4: Kiểm Tra Cảm Biến MPU6050 & Encoder GA25 (`sensor_test_live.py`)](#6-module-4-kiểm-tra-cảm-biến-mpu6050--encoder-ga25-sensor_test_livepy)
7. [Module 5: Bộ Công Cụ Chẩn Đoán & Tinh Chỉnh Toàn Diện (`hardware_check.py`)](#7-module-5-bộ-công-cụ-chẩn-đoán--tinh-chỉnh-toàn-diện-hardware_checkpy)

---

## 1. KIẾN TRÚC & PHÂN VAI PHẦN CỨNG

Hệ thống sử dụng **2 vi điều khiển riêng biệt** với vai trò rõ ràng:

```text
  [ Điện Thoại / PC ]
          │ (Bluetooth 5.0 LE / Cáp USB)
          ▼
┌─────────────────────────────────┐
│     ESP32-S3 DEVKIT             │  <─── VAI TRÒ: MODULE BLUETOOTH / KHÔNG DÂY
│ (Có sẵn chip Bluetooth & WiFi)  │       (Thay thế module Bluetooth HC-05 cũ)
└────────────────┬────────────────┘
                 │ (2 dây UART: Chân 4 & 5 @ 115200bps)
                 ▼
┌─────────────────────────────────┐
│     STM32F401 BLACKPILL         │  <─── VAI TRÒ: BỘ NÃO TRUNG TÂM (MAIN CONTROLLER)
│ (ARM Cortex-M4 84MHz, FPU)      │       • Đọc cảm biến MPU6050 (I2C)
│                                 │       • Đọc 2 đĩa Encoder GA25 (TIM2, TIM3)
│ (STM32 KHÔNG CÓ Bluetooth!)     │       • Chạy bộ lọc Kalman + Cascade PID 100Hz
└────────────────┬────────────────┘       • TRỰC TIẾP XUẤT PWM LÁI 2 ĐỘNG CƠ!
                 │ (PA4..PA7, PB8, PB9, PB0)
                 ▼
┌─────────────────────────────────┐
│     MẠCH CẦU H TB6612FNG        │  <─── Mạch công suất nhận lệnh từ STM32
└────────────────┬────────────────┘
                 ▼
        [ 2 ĐỘNG CƠ GA25 ]
```

---

## 2. SƠ ĐỒ ĐẤU DÂY CHUẨN TOÀN HỆ THỐNG

### 2.1. Kết nối Động cơ & Mạch cầu H TB6612 $\longleftrightarrow$ STM32

| Chân TB6612 | Chân STM32 BlackPill | Nguồn Điện | Chức năng |
| :--- | :--- | :--- | :--- |
| **PWMA** | **PB8** (TIM4_CH3) | Logic | Xung PWM điều khiển tốc độ Động cơ Trái (0 - 4199) |
| **AIN1** | **PA4** | Logic | Chiều quay Động cơ Trái (Chân 1) |
| **AIN2** | **PA5** | Logic | Chiều quay Động cơ Trái (Chân 2) |
| **PWMB** | **PB9** (TIM4_CH4) | Logic | Xung PWM điều khiển tốc độ Động cơ Phải (0 - 4199) |
| **BIN1** | **PA6** | Logic | Chiều quay Động cơ Phải (Chân 1) |
| **BIN2** | **PA7** | Logic | Chiều quay Động cơ Phải (Chân 2) |
| **STBY** | **PB0** (hoặc nối 3.3V) | Logic | Chốt an toàn (Mức 1 = Mở công suất, Mức 0 = Phanh) |
| **VCC** | **3.3V hoặc 5V** | Nguồn bo | Nguồn nuôi chip logic TB6612 |
| **VM** | **Cực Dương (+) Pin** | **7.4V - 11.1V** | **Nguồn cấp lực quay động cơ (Bắt buộc dùng Pin)** |
| **GND** | **GND** | Mass | Nối chung mass với STM32 và Cực Âm (-) Pin |

### 2.2. Kết nối Cảm biến MPU6050 $\longleftrightarrow$ STM32

| Chân MPU6050 | Chân STM32 BlackPill | Chức năng |
| :--- | :--- | :--- |
| **VCC** | **3.3V** (hoặc 5V) | Nguồn nuôi cảm biến |
| **GND** | **GND** | Nối Mass |
| **SCL** | **PB6** (I2C1_SCL) | Xung nhịp I2C (Cần trở kéo pull-up 4.7k nếu dùng board rời) |
| **SDA** | **PB7** (I2C1_SDA) | Dữ liệu I2C |

### 2.3. Kết nối Đĩa Encoder GA25 $\longleftrightarrow$ STM32

| Động cơ GA25 | Chân STM32 BlackPill | Chức năng |
| :--- | :--- | :--- |
| **Encoder Trái - Phase A** | **PA0** (TIM2_CH1) | Đếm xung kênh A motor trái |
| **Encoder Trái - Phase B** | **PA1** (TIM2_CH2) | Đếm xung kênh B motor trái |
| **Encoder Phải - Phase A** | **PB4** (TIM3_CH1) | Đếm xung kênh A motor phải |
| **Encoder Phải - Phase B** | **PB5** (TIM3_CH2) | Đếm xung kênh B motor phải |
| **VCC Encoder** | **3.3V** | Nguồn nuôi mắt đọc quang/từ tính |
| **GND Encoder** | **GND** | Nối Mass |

### 2.4. Kết nối ESP32-S3 (Bridge) $\longleftrightarrow$ STM32 (Main)

| Chân ESP32-S3 | Hướng tín hiệu | Chân STM32 BlackPill | Chức năng |
| :--- | :---: | :--- | :--- |
| **Chân số 4** (GPIO 4) | `--->` | **Chân A10** (USART1_RX) | Truyền lệnh từ điện thoại/PC xuống xe |
| **Chân số 5** (GPIO 5) | `<---` | **Chân A9** (USART1_TX) | Gửi dữ liệu góc/tốc độ từ xe lên điện thoại/PC |
| **GND** | `<--->` | **GND** | Bắt buộc nối chung Mass |

---

## 3. MODULE 1: KIỂM TRA MẠCH CẦU NỐI ESP32-S3 (`test_esp32_loopback.py`)

### Mục đích:
Xác minh độc lập chip ESP32-S3, cổng nạp USB và xác định chính xác vị trí chân GPIO 4 và 5 trên bo mạch (không cần cắm vào STM32).

### Cách thực hiện:
1. Rút 2 dây đang nối sang STM32 ra.
2. Lấy **1 sợi dây cắm nối tắt trực tiếp chân `4` vào chân `5`** ngay trên mạch ESP32-S3.
3. Chạy lệnh:
```bash
./venv/bin/python3 Tools/test_esp32_loopback.py
```
- **Kết quả thành công**: Hiện thông báo `[THÀNH CÔNG] ESP32 ĐÃ PHẢN HỒI LOOPBACK CHÍNH XÁC`. Khẳng định ESP32 và chân 4, 5 hoạt động 100%!
- **Kết quả thất bại**: Bạn đã cắm nhầm chân (hãy tìm chân in số `4` và `5` ở hàng chân bên trái, **không cắm** vào chân in chữ TX/RX).

---

## 4. MODULE 2: KIỂM TRA THÔNG TUYẾN UART VẬT LÝ (`test_bridge_live.py`)

### Mục đích:
Kiểm tra tín hiệu 2 chiều giữa ESP32 và STM32 (dây nối, tốc độ Baud 115200bps, tiếp xúc chân).

### Cách thực hiện:
1. Nối 3 dây từ ESP32 sang STM32 theo bảng 2.4 (Chân 4 $\rightarrow$ A10, Chân 5 $\rightarrow$ A9, GND $\rightarrow$ GND).
2. Chạy lệnh:
```bash
./venv/bin/python3 Tools/test_bridge_live.py
```
3. Trong lúc script đang chạy, **bấm nhả nút `NRST` (Reset)** trên mạch STM32:
- **Kết quả thành công**: Màn hình hiện dòng khởi động màu xanh `★ [BOOTLOADER/SYS BANNER]: [SYS] MPU6050: OK` và phản hồi `✔ [PHẢN HỒI THÀNH CÔNG TỪ STM32]: PONG`.
- **Nếu không thấy gì**: Hãy tráo đổi vị trí 2 đầu dây ở chân 4 và 5 trên ESP32.

---

## 5. MODULE 3: KIỂM TRA ĐỘNG CƠ & MẠCH CẦU H TB6612

> [!IMPORTANT]
> **Yêu cầu an toàn**: Kê xe lên giá đỡ để bánh xe quay tự do trong không khí. Phải cấp nguồn Pin (7.4V) vào chân **`VM`** và **`GND`** của TB6612. Không cấp nguồn động cơ từ dây USB!

### Cách 5.1: Chế độ Test Tự Động trên STM32 (Không cần máy tính)
Mã nguồn hỗ trợ chế độ tự chạy chu trình test 6 pha ngay khi khởi động.
1. Mở file [Core/Src/main.c](file:///home/cbg6682/Documents/STM32/self_balancing_car/Core/Src/main.c#L49), đặt:
```c
#define RUN_MOTOR_TEST_MODE 1
```
2. Biên dịch và nạp code vào STM32:
```bash
ninja -C build/Debug
./venv/bin/python3 Tools/flash_hid.py build/Debug/self_balancing_car.bin
```
3. **Hiện tượng kiểm tra**:
   - Pha 1: Bánh TRÁI quay TIẾN (1.2s) $\rightarrow$ Dừng 0.5s.
   - Pha 2: Bánh TRÁI quay LÙI (1.2s) $\rightarrow$ Dừng 0.5s.
   - Pha 3: Bánh PHẢI quay TIẾN (1.2s) $\rightarrow$ Dừng 0.5s.
   - Pha 4: Bánh PHẢI quay LÙI (1.2s) $\rightarrow$ Dừng 0.5s.
   - Pha 5: CẢ 2 BÁNH đồng thời quay TIẾN (1.5s) $\rightarrow$ Dừng 0.5s.
   - Pha 6: CẢ 2 BÁNH đồng thời quay LÙI (1.5s) $\rightarrow$ Nghỉ 2s $\rightarrow$ Lặp lại.

*(Khi muốn chuyển lại chế độ tự cân bằng, chỉ cần sửa lại `#define RUN_MOTOR_TEST_MODE 0` rồi nạp lại).*

### Cách 5.2: Bảng Điều Khiển Tương Tác từ Máy Tính (`motor_test_interactive.py`)
Khi đã thông tuyến UART, chạy công cụ điều khiển bằng bàn phím:
```bash
./venv/bin/python3 Tools/motor_test_interactive.py
```
- Phím `1`, `2`: Điều khiển Motor Trái Tiến / Lùi.
- Phím `3`, `4`: Điều khiển Motor Phải Tiến / Lùi.
- Phím `5`, `6`: Điều khiển Cả 2 Motor Tiến / Lùi.
- Phím `7`: Chạy chu trình tự động 6 pha.
- Phím `8`: Nhập tốc độ PWM tùy chọn (ví dụ: `2500,-2500`).
- Phím `s`: Dừng khẩn cấp.

---

## 6. MODULE 4: KIỂM TRA CẢM BIẾN MPU6050 & ENCODER GA25 (`sensor_test_live.py`)

### Mục đích:
Xem trực quan dữ liệu cảm biến thời gian thực, kiểm tra độ nhạy và chiều dương/âm của góc nghiêng và đĩa encoder.

### Cách thực hiện:
Chạy lệnh trong terminal:
```bash
./venv/bin/python3 Tools/sensor_test_live.py
```
Màn hình hiển thị đồng hồ đo trực quan:
```text
Pitch:  +2.14° [         |==●          ] | Gyro:   -0.8°/s | EncL:     1250 | EncR:     1248 | BALANCING (Cân bằng)
```
- **Kiểm tra MPU6050**: Nghiêng xe về phía trước $\rightarrow$ Pitch phải tăng dương (`+`). Nghiêng xe về sau $\rightarrow$ Pitch phải âm (`-`).
- **Kiểm tra Encoder Trái**: Lấy tay quay bánh xe trái về phía trước $\rightarrow$ `EncL` phải tăng lên.
- **Kiểm tra Encoder Phải**: Lấy tay quay bánh xe phải về phía trước $\rightarrow$ `EncR` phải tăng lên.

---

## 7. MODULE 5: BỘ CÔNG CỤ CHẨN ĐOÁN & TINH CHỈNH TOÀN DIỆN (`hardware_check.py`)

Bộ công cụ mạnh nhất để cân chỉnh thông số phần cứng trước khi chạy thực tế:
```bash
./venv/bin/python3 Tools/hardware_check.py
```
Menu các tính năng chuyên sâu:
1. **Live IMU Visualizer**: Hiển thị thước đo cân bằng và tốc độ góc 100Hz.
2. **Encoder Verification**: Kiểm tra chiều quay và số xung của encoder bánh xe.
3. **Motor Test Pulse**: Phát xung test ngắn để đo vùng chết động cơ (Deadzone).
4. **Invert Polarity**: Đảo chiều quay động cơ/encoder trực tiếp qua phần mềm mà không cần hàn lại dây!
5. **Zero Balance Trim**: Đặt góc 0 tự cân bằng (bù trọng tâm xe khi gắn pin lệch).
6. **PID Auto-Tuning**: Tự động thu thập dữ liệu và tối ưu bộ điều khiển PID theo ITAE.
7. **Virtual Simulator**: Chạy mô phỏng RK4 khi không cắm phần cứng thật.

---

## 8. TƯƠNG THÍCH ĐA NỀN TẢNG (LINUX, WINDOWS & MACOS)

Tất cả các công cụ kiểm tra đều được viết bằng Python chuẩn với thư viện `pyserial`, `hidapi` và `logger_utils`, hoạt động **100% đồng nhất trên cả Linux, Windows và macOS**:

- **Tự động nhận diện cổng Serial**:
  - **Linux**: Tự nhận `/dev/ttyACM0`, `/dev/ttyUSB0`...
  - **Windows**: Tự nhận cổng `COM3`, `COM4`, `COM5`... (chip CH343/CH340/CP2102).
  - Hoặc có thể truyền thủ công cổng vào tham số lệnh (ví dụ: `python Tools\motor_test_interactive.py COM4`).
- **Hỗ trợ màu sắc ANSI**: Tự động kích hoạt chế độ Windows VT100 terminal (`cmd.exe` và `PowerShell`).

### Bảng đối chiếu lệnh chạy Linux vs Windows:

| Chức năng kiểm tra | Trên Linux / macOS (Bash) | Trên Windows (CMD / PowerShell) |
| :--- | :--- | :--- |
| **Kiểm tra Tổng Thể có Xác Nhận** | `./run_pipeline.sh --selftest` | `run_pipeline.bat --selftest` |
| **Test Loopback ESP32** | `./run_pipeline.sh --check` | `run_pipeline.bat --check` |
| **Test thông tuyến UART** | `./run_pipeline.sh --bridge` | `run_pipeline.bat --bridge` |
| **Test Động cơ tương tác** | `./run_pipeline.sh --motor` | `run_pipeline.bat --motor` |
| **Bảng cảm biến & Encoder** | `./run_pipeline.sh --sensor` | `run_pipeline.bat --sensor` |
| **Cân chỉnh & Tune toàn diện** | `./run_pipeline.sh --hardware` | `run_pipeline.bat --hardware` |
| **Nạp firmware USB HID (Type-C)** | `./run_pipeline.sh --flash` | `run_pipeline.bat --flash` |
| **Nạp firmware qua ST-Link (SWD)** | `./run_pipeline.sh --stlink` | `run_pipeline.bat --stlink` |

---

## TỔNG KẾT BẢNG LỆNH NHANH

| Mục tiêu kiểm tra | Lệnh chạy trực tiếp (Python) | Lệnh qua Pipeline script |
| :--- | :--- | :--- |
| **Kiểm tra TỔNG THỂ có xác nhận** | `python Tools/system_self_test.py` | `./run_pipeline.sh --selftest` |
| **Test cổng ESP32 & Chân 4, 5** | `python Tools/test_esp32_loopback.py` | `./run_pipeline.sh --check` |
| **Test bắt tay UART STM32 <-> ESP32** | `python Tools/test_bridge_live.py` | `./run_pipeline.sh --bridge` |
| **Test cảm biến MPU6050 & Encoder** | `python Tools/sensor_test_live.py` | `./run_pipeline.sh --sensor` |
| **Test động cơ TB6612 tương tác** | `python Tools/motor_test_interactive.py` | `./run_pipeline.sh --motor` |
| **Chẩn đoán & Cân chỉnh toàn hệ thống** | `python Tools/hardware_check.py` | `./run_pipeline.sh --hardware` |
| **Nạp firmware STM32 (USB HID)** | `python Tools/flash_hid.py build/Debug/self_balancing_car.bin` | `./run_pipeline.sh --flash` |
| **Nạp firmware STM32 (ST-Link)** | `python Tools/flash_stlink.py build/Debug/self_balancing_car.bin` | `./run_pipeline.sh --stlink` |

# Hướng Dẫn Sử Dụng ESP32-S3 DevKitC Làm Mạch Bluetooth (BLE 5.0) Rời Cho STM32

Tài liệu này hướng dẫn cách cấu hình, nạp code và đấu nối bo mạch **ESP32-S3 DevKitC** làm mạch cầu nối **Bluetooth Low Energy (BLE 5.0)** tốc độ cao (Nordic UART Service) cho Xe Hai Bánh Tự Cân Bằng STM32F401CCU6, đồng thời hỗ trợ giám sát/thu thập dữ liệu qua cổng USB-CDC trên máy tính.

---

## 1. So Sánh Kỹ Thuật: HC-05 vs. ESP32-S3 BLE 5.0

| Tiêu Chí | Module HC-05 (Cũ) | Module ESP32-S3 DevKitC (Hiện tại) |
| :--- | :--- | :--- |
| **Chuẩn Bluetooth** | Bluetooth 2.0 + EDR (Classic SPP) | **Bluetooth 5.0 Low Energy (BLE NUS)** |
| **Mức điện áp Logic** | Thường cần trở chia áp cho chân RX | **3.3V thuần — nối thẳng STM32F401, không cần chia áp** |
| **Bảo vệ mất kết nối (Fail-safe)** | Thụ động | **Chủ động gửi `BLE:DISCONNECTED` để STM32 tự phanh đứng yên** |
| **Hỗ trợ nền tảng** | Chủ yếu Android | **Android, iOS, Windows, Linux, macOS** |
| **Độ trễ & USB Debug** | Trễ ~30–50 ms, không có USB bridge | **Độ trễ < 10 ms, tích hợp cầu USB-CDC nối thẳng PC (`/dev/ttyACM0`)** |
| **Nguồn cấp** | 5V @ 30–50 mA | **5V @ 300–500 mA (từ Buck 5V hoặc cổng USB)** |

---

## 2. Sơ Đồ Đấu Nối Phần Cứng (Wiring Pinout)

Khớp chính xác với định nghĩa `PIN_TO_STM32_RX = 6` và `PIN_FROM_STM32_TX = 3` trong [`esp32_s3_ble_bridge.ino`](esp32_s3_ble_bridge.ino):

```text
 +--------------------------+                 +--------------------------+
 |   ESP32-S3 DevKitC-1     |                 |  STM32F401CCU6 BlackPill |
 |                          |                 |                          |
 |  GPIO 6 (UART1 TX)       |---------------->|  PA10 (USART1 RX)        |
 |  GPIO 3 (UART1 RX)       |<----------------|  PA9  (USART1 TX)        |
 |  GND                     |<--------------->|  GND  (Mass chung)       |
 |  5V (hoặc VIN)           |<--- 5V Buck ----|  5V   (Nguồn Buck 5V)    |
 +--------------------------+                 +--------------------------+
```

> [!IMPORTANT]
> 1. **Đấu chéo UART:** `GPIO 6` (TX của ESP32-S3) nối vào `PA10` (RX của STM32); `GPIO 3` (RX của ESP32-S3) nối vào `PA9` (TX của STM32).
> 2. **Điện áp Logic 3.3V:** Cả hai MCU dùng logic 3.3V, nối trực tiếp không cần mạch đệm.
> 3. **Nguồn cấp:** Cấp `5V` từ mạch Buck (≥ 500 mA) hoặc USB Type-C; **không** lấy từ chân `3.3V` của BlackPill.

---

## 3. Hướng Dẫn Nạp Code Lên ESP32-S3 DevKitC

1. Cài **Arduino IDE 2.x** và cài gói bo mạch `esp32` của Espressif Systems trong *Boards Manager*.
2. Mở file [`esp32_s3_ble_bridge.ino`](esp32_s3_ble_bridge.ino).
3. Cấu hình bo mạch:
   * **Board:** `ESP32S3 Dev Module`
   * **USB CDC On Boot:** `Enabled`
   * **Upload Speed:** `921600`
   * **Port:** Chọn đúng cổng COM / `/dev/ttyACM*` của ESP32-S3.
4. Nhấn **Upload**.

---

## 4. Hướng Dẫn Kết Nối Điện Thoại

* **Android:** Cài app **Serial Bluetooth Terminal** (*Kai Morich*) → *Devices* → tab **Bluetooth LE** → *Scan* → chọn **`STM32_Balance_Car`** → *Connect*.
* **iOS:** Cài **BLE Terminal HM-10** hoặc **nRF Connect for Mobile** → quét và kết nối **`STM32_Balance_Car`**.

---

## 5. Bảng Mã Lệnh Điều Khiển & Tinh Chỉnh Qua Bluetooth (BLE)

Toàn bộ lệnh được STM32 giải mã tại [`Modules/Src/telemetry_uart.c`](../../Modules/Src/telemetry_uart.c) (không phân biệt hoa/thường):

| Lệnh ngắn | Lệnh đầy đủ | Chức năng |
| :---: | :--- | :--- |
| **`F`** | `FORWARD` | Chạy tiến êm (`+0.15 m/s`, có ramp mượt; tự dừng sau `500 ms` nếu nhả nút) |
| **`B`** | `BACKWARD` | Chạy lùi êm (`-0.15 m/s`, có ramp mượt) |
| **`L`** | `LEFT` | Quay tại chỗ sang trái (`turn_offset = -150`) |
| **`R`** | `RIGHT` | Quay tại chỗ sang phải (`turn_offset = +150`) |
| **`S`** | `HALT` | Dừng di chuyển, giữ thăng bằng đứng yên tại chỗ |
| **`AUTO`** | `START` / `RUN` | Bật chế độ tự cân bằng (tự chạy khi dựng thẳng trong `±10°` đủ `0.2 s`) |
| **`OFF`** | `STOP` | Ngắt khẩn cấp động cơ (`STBY = LOW`, `PWM = 0`) |
| **`PARAMS`** | `INFO` / `STATUS` | Hiển thị đầy đủ PID, Offset, Deadzone, chế độ, Pitch, Gyro, vận tốc, Encoder |
| **`SET_PID:...`** | `SET_PID:kp,ki,kd,skp,ski` | Nạp nóng 5 hệ số PID vào RAM (VD: `SET_PID:40,0,1.4,28,2.5`) |
| **`GET_PID`** | `PID?` | Xem 5 hệ số PID hiện tại |
| **`OFFSET:...`** | `OFFSET:<deg>` | Chỉnh nóng góc cân bằng cơ khí (VD: `OFFSET:-3.60`) |
| **`DEADZONE:...`** | `DEADZONE:<val>` | Dịch vùng chết 2 bánh (`0..400`, giữ nguyên độ lệch Trái/Phải) |
| **`RESET_ENC`** | `ZERO_ENC` | Đặt lại bộ đếm xung 2 encoder về `(0, 0)` |
| **`PROBE`** | `PROBE:A` / `PROBE:B` / `PROBE:PWM` | Khóa `STBY = 0V`, kích chân điều khiển lên `3.3V` để đo VOM an toàn |
| **`MOTOR:l,r`** | `MOTOR:200,200` | Chạy thử trực tiếp 2 bánh qua bù Deadzone & Scale |
| **`START_STREAM`** | `STOP_STREAM` | Bật / tắt luồng telemetry 100 Hz (`DATA:...` + `DBG:...`) |
| **`PING`** | `ESP:PING` | Kiểm tra kết nối (`PONG` / `ESP:PONG`) |
| **`HELP`** | `?` | In nhanh danh sách lệnh trên điện thoại |

> [!TIP]
> Gán 5 nút Macro trong app **Serial Bluetooth Terminal** thành `F`, `B`, `L`, `R`, `S` (và thêm 2 nút `AUTO`, `OFF`) để điều khiển xe thuận tiện như tay cầm từ xa.

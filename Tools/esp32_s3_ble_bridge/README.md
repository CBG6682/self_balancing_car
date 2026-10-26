# Hướng Dẫn Sử Dụng ESP32-S3 DevKitC Làm Mạch Bluetooth (BLE 5.0) Rời Cho STM32

Tài liệu này hướng dẫn chi tiết cách cấu hình, nạp code và đấu nối bo mạch **ESP32-S3 DevKitC** để thay thế hoàn toàn module Bluetooth cổ điển **HC-05**, biến ESP32-S3 thành một module Bluetooth Low Energy (BLE 5.0) rời tốc độ cao cho Xe Hai Bánh Tự Cân Bằng STM32F401CCU6.

---

## 1. So Sánh Kỹ Thuật: HC-05 vs. ESP32-S3 BLE 5.0

| Tiêu Chí | Module HC-05 (Cũ) | Module ESP32-S3 DevKitC (Mới) |
| :--- | :--- | :--- |
| **Chuẩn Bluetooth** | Bluetooth 2.0 + EDR (Classic SPP) | **Bluetooth 5.0 Low Energy (BLE)** |
| **Mức điện áp Logic** | 3.3V (nhưng thường cấp 5V, cần cầu phân áp RX) | **3.3V thuần (Khớp 100% với STM32F401, không cần trở chia áp)** |
| **Bảo vệ mất kết nối (Fail-safe)** | Thụ động (không báo ngắt trừ khi nối chân STATE) | **Chủ động gửi `BLE:DISCONNECTED` khiến xe tự động phanh gấp** |
| **Hỗ trợ iOS / Android / Web** | Chỉ hỗ trợ Android (iOS không hỗ trợ SPP 2.0) | **Hỗ trợ toàn diện: Android, iOS, Windows, macOS, Web Bluetooth** |
| **Băng thông & Độ trễ** | Tối đa ~20-30 kB/s, trễ ~30-50ms | **Tốc độ cao 2Mbps PHY, MTU 256 bytes, trễ cực thấp < 10ms** |
| **Nguồn cấp yêu cầu** | 5V @ 30-50mA | **5V @ 300-500mA (Lấy từ mạch hạ áp Buck LM2596 hoặc cổng USB)** |

---

## 2. Sơ Đồ Đấu Nối Phần Cứng (Wiring Pinout)

Bo mạch ESP32-S3 DevKitC được cắm rời và giao tiếp với STM32 qua chuẩn UART (Serial1):

```text
 +--------------------------+                 +--------------------------+
 |   ESP32-S3 DevKitC-1     |                 |  STM32F401CCU6 BlackPill |
 |                          |                 |                          |
 |  GPIO 4 (UART1 TX)       |---------------->|  PA10 (USART1 RX)        |
 |  GPIO 5 (UART1 RX)       |<----------------|  PA9  (USART1 TX)        |
 |  GND                     |<--------------->|  GND  (Mass chung)       |
 |  5V (hoặc VIN)           |<--- 5V Pin/BEC -|  5V   (Nguồn Buck LM2596)|
 +--------------------------+                 +--------------------------+
```

> [!IMPORTANT]
> 1. **Dây chéo UART**: Chân TX của ESP32-S3 nối vào chân RX của STM32, và ngược lại.
> 2. **Điện áp Logic**: Cả STM32F401 và ESP32-S3 đều sử dụng mức logic 3.3V, nối trực tiếp dây cắm Dupont **không cần** bất kỳ mạch đệm hay trở phân áp nào.
> 3. **Nguồn cấp**: Nối chân `5V` (hoặc `VIN`) của ESP32-S3 vào đường nguồn 5V từ mạch hạ áp LM2596 (hoặc cắm cáp Type-C riêng). **Không** lấy nguồn từ chân 3.3V của STM32 vì bộ ổn áp LDO trên mạch Black Pill chỉ chịu tải ~100mA, không đủ dòng phát sóng RF của ESP32-S3 (~300-500mA peak).

---

## 3. Hướng Dẫn Nạp Code Lên ESP32-S3 DevKitC

### Cách 1: Sử dụng Arduino IDE (Khuyến nghị)
1. Cài đặt **Arduino IDE 2.x**.
2. Thêm URL quản lý bo mạch ESP32 vào *File -> Preferences -> Additional Boards Manager URLs*:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Vào *Tools -> Board -> Boards Manager*, tìm `esp32` của Espressif Systems và bấm **Install**.
4. Mở file [Tools/esp32_s3_ble_bridge/esp32_s3_ble_bridge.ino](esp32_s3_ble_bridge.ino).
5. Chọn cấu hình bo mạch:
   - **Board**: `"ESP32S3 Dev Module"`
   - **USB CDC On Boot**: `"Enabled"`
   - **Flash Size**: `"4MB (32Mb)"` hoặc `"8MB"` tùy bản DevKitC bạn mua
   - **Upload Speed**: `921600`
   - **Port**: Chọn cổng COM tương ứng của ESP32-S3
6. Bấm **Upload**.

---

## 4. Hướng Dẫn Kết Nối Điện Thoại (App Điều Khiển)

Vì ESP32-S3 sử dụng chuẩn **Bluetooth 5.0 Low Energy (BLE)** theo chuẩn dịch vụ **Nordic UART Service (NUS)**, việc kết nối thực hiện trực tiếp trong ứng dụng mà không cần vào cài đặt Bluetooth quét mã PIN như HC-05.

### 4.1. Trên điện thoại Android
1. Cài đặt app **"Serial Bluetooth Terminal"** (Tác giả: *Kai Morich* - miễn phí trên Google Play).
2. Mở app, vuốt menu bên trái chọn **Devices** $\rightarrow$ chuyển sang tab **Bluetooth LE**.
3. Bấm quét (Scan) $\rightarrow$ Chọn thiết bị tên **`STM32_Balance_Car`** $\rightarrow$ Bấm **Connect**.
4. Màn hình báo *Connected*! Mọi ký tự gõ gửi đi sẽ được chuyển trực tiếp tới STM32.

### 4.2. Trên điện thoại iOS (iPhone / iPad)
1. Cài đặt app **"BLE Terminal HM-10"** hoặc **"nRF Connect for Mobile"**.
2. Quét thiết bị **`STM32_Balance_Car`** và bấm kết nối.

---

## 5. Bảng Mã Lệnh Điều Khiển Qua Bluetooth (BLE)

Các lệnh này được STM32 giải mã trực tiếp trong file `Modules/Src/telemetry_uart.c`:

| Ký Tự Lệnh | Lệnh Dài Tương Đương | Ý Nghĩa / Hành Động Của Xe |
| :---: | :--- | :--- |
| **`F`** | `FORWARD` hoặc `CMD:FORWARD` | Tiến về phía trước (+0.35 m/s) |
| **`B`** | `BACKWARD` hoặc `CMD:BACKWARD`| Lùi về phía sau (-0.35 m/s) |
| **`L`** | `LEFT` hoặc `CMD:LEFT` | Quay trái tại chỗ (Offset $\Delta PWM = -600$) |
| **`R`** | `RIGHT` hoặc `CMD:RIGHT` | Quay phải tại chỗ (Offset $\Delta PWM = +600$) |
| **`S`** | `STOP` hoặc `CMD:STOP` | Phanh dừng khẩn cấp và giữ thăng bằng tại chỗ |
| **`PING`** | `PING` | Kiểm tra bắt tay (STM32 trả về `PONG`) |
| **`STATUS`**| `CMD:STATUS` | Trả về trạng thái máy FSM và trạng thái BLE |
| - | `START_STREAM` | Bật truyền định kỳ dữ liệu Telemetry (`DATA:...`) |
| - | `STOP_STREAM` | Tắt truyền dữ liệu Telemetry |
| - | `SET_PID:kp,ki,kd,skp,ski` | Nạp nóng bộ thông số Cascade PID vào RAM |

> [!TIP]
> Bạn có thể gán 5 nút bấm nhanh (Macro Buttons) trong app **Serial Bluetooth Terminal** thành các chữ cái `F`, `B`, `L`, `R`, `S` để điều khiển xe như tay cầm từ xa. Khi buông tay hoặc ngắt kết nối Bluetooth, xe sẽ tự động kích hoạt tính năng tự phanh để giữ thăng bằng an toàn.

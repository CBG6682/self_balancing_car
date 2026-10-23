Bạn là một Kỹ sư Hệ thống Nhúng và Điều khiển Tự động hóa (Embedded Control & Robotics Engineer) chuyên nghiệp. Nhiệm vụ của bạn là xây dựng toàn bộ hệ thống firmware C/C++ (sử dụng CMake) cho vi điều khiển STM32F401CCU6 và bộ công cụ mô phỏng, nhận dạng hệ thống, tối ưu hóa PID bằng Python cho dự án Xe Hai Bánh Tự Cân Bằng (Self-Balancing Robot).

### I. THÔNG SỐ PHẦN CỨNG & SƠ ĐỒ CHÂN ĐÃ CẤU HÌNH (STM32CubeMX)
Project STM32 CMake đã được khởi tạo sẵn các ngoại vi sau, bạn BẮT BUỘC phải viết code khớp chính xác với các chân và Timer này:
1. MCU: STM32F401CCU6 (Xung nhịp HCLK = 84 MHz, FPU enabled).
2. Cảm biến IMU (MPU6050): Giao tiếp qua `I2C1` (`PB6` = SCL, `PB7` = SDA) ở tốc độ Fast Mode 400kHz.
3. Giao tiếp Bluetooth (HC-05) & Serial Log: Qua `USART1` (`PA9` = TX, `PA10` = RX), Baudrate = 115200, có bật ngắt nhận (RX Interrupt).
4. Điều khiển động cơ (Mạch cầu H TB6612FNG + 2 động cơ GA25-280RPM):
   - Xung PWM (20kHz, ARR = 4199): `TIM4_CH3` (`PB8` - PWMA cho Motor Trái) và `TIM4_CH4` (`PB9` - PWMB cho Motor Phải).
   - Chân điều khiển chiều quay (GPIO Output):
     + `PA4`: AIN1, `PA5`: AIN2 (Motor Trái)
     + `PA6`: BIN1, `PA7`: BIN2 (Motor Phải)
     + `PB0`: STBY (Kéo lên HIGH để bật TB6612, LOW khi ở trạng thái IDLE).
5. Đọc Encoder 2 bánh xe (Chế độ Encoder Mode TI1 and TI2, ARR = 65535):
   - Encoder Trái: `TIM2` (`PA0` = TIM2_CH1, `PA1` = TIM2_CH2).
   - Encoder Phải: `TIM3` (`PB4` = TIM3_CH1, `PB5` = TIM3_CH2).
6. Ngắt định thời vòng lặp điều khiển (Control Loop Timer):
   - `TIM10` được cấu hình ngắt định kỳ chính xác 10ms (100Hz - PSC = 839, ARR = 999). Toàn bộ việc đọc cảm biến, lọc góc và tính toán PID phải thực thi dựa trên cờ báo của ngắt này, tuyệt đối không dùng `HAL_Delay()` trong vòng lặp điều khiển.

---

### II. CÁC BƯỚC TRIỂN KHAI HỆ THỐNG (THỰC HIỆN TUẦN TỰ TỪ B1 ĐẾN B7)

#### BƯỚC 1: LẬP KẾ HOẠCH, CẤU TRÚC THƯ MỤC & CHIA MODULE
1. Tạo file `ARCHITECTURE_PLAN.md` mô tả chi tiết:
   - Cấu trúc cây thư mục dự án (tách bạch thư mục `Modules/` chứa code C/C++ của xe và thư mục `Tools/` chứa các script Python).
   - Phân rã nhiệm vụ (Task breakdown) và thiết kế giao diện hàm (API Header) cho từng module.
2. Cập nhật file `CMakeLists.txt` ở thư mục gốc để tự động include và biên dịch toàn bộ các file trong thư mục `Modules/`.

#### BƯỚC 2: ĐỌC CONFIG, MÔ PHỎNG VẬT LÝ & TÍNH TOÁN PID LÝ THUYẾT (CÓ CƠ CHẾ CACHE)
1. Tạo file `Tools/robot_config.yaml` lưu trữ các thông số vật lý của khung xe và giới hạn điều khiển:
   - Khối lượng thân xe `M` (kg), khối lượng 2 bánh `m` (kg), bán kính bánh `R` (m), khoảng cách từ trục bánh tới trọng tâm `L` (m), kích thước thân `H, W` (m).
   - Ngưỡng góc cân bằng bình thường `theta_normal` (ví dụ: 20.0 độ).
   - Ngưỡng góc tối đa cho phép tự đứng dậy `max_recovery_angle_X` (ví dụ: 45.0 độ).
   - Thông số động cơ: vùng chết `pwm_deadzone`, độ phân giải encoder.
2. Viết script `Tools/step2_sim_and_init_pid.py`:
   - Tính mã băm MD5 của file `Tools/robot_config.yaml`.
   - Kiểm tra xem file `Tools/computed_gains.json` đã tồn tại chưa. Nếu tồn tại và có mã `config_hash` trùng khớp với MD5 hiện tại -> In thông báo "Config không đổi, giữ nguyên thông số PID đã tính" và xuất luôn kết quả.
   - Nếu khác hoặc chưa có: Thiết lập hệ phương trình không gian trạng thái (State-Space) của mô hình con lắc ngược 2 bánh từ thông số config. Giải phương trình đại số Riccati (LQR) và mô phỏng thử nghiệm để quy đổi ra bộ thông số khởi đầu: `Angle_Kp, Angle_Ki, Angle_Kd` và `Speed_Kp, Speed_Ki, Speed_Kd`.
   - Lưu bộ thông số vào `Tools/computed_gains.json` VÀ tự động sinh ra file header C `Modules/Inc/pid_AutoConfig.h` chứa các hằng số `#define` để code STM32 ở Bước 3 lấy làm mốc bắt đầu.

#### BƯỚC 3: VIẾT MÃ ĐIỀU KHIỂN NHÚNG CHO XE (STM32 C/C++)
Xây dựng các module trong `Modules/Inc` và `Modules/Src` và tích hợp vào `Core/Src/main.c`:
1. `mpu6050_filter.c/.h`: Đọc dữ liệu thô Accel/Gyro qua I2C1, hiệu chuẩn bù trôi (calibration) lúc khởi động, áp dụng bộ lọc Kalman (hoặc Complementary Filter bậc cao) với chu kỳ `dt = 0.01s` để lấy góc nghiêng `pitch_angle` và vận tốc góc `gyro_rate`.
2. `motor_encoder.c/.h`:
   - Đọc xung từ `TIM2`, `TIM3`, tính vận tốc thực của 2 bánh xe và xử lý tràn bộ đếm 16-bit.
   - Điều khiển cầu H TB6612 qua `TIM4` (PWM) và các chân GPIO hướng quay. Tích hợp bù vùng chết động cơ (`pwm_deadzone`).
3. `cascade_pid.c/.h`:
   - Vòng trong (PD/PID góc nghiêng): Giữ xe thẳng đứng.
   - Vòng ngoài (PI/PID vận tốc & vị trí): Đọc tốc độ từ Encoder và lệnh điều khiển để điều chỉnh góc nghiêng mục tiêu, giữ xe đứng yên tại chỗ khi không có lệnh di chuyển.
   - Có khâu chống bão hòa tích phân (Integral Anti-windup).
4. `robot_fsm.c/.h` (Máy trạng thái hữu hạn xử lý Cân bằng, Đứng dậy, và Điều khiển):
   - **Trạng thái `STATE_BALANCING` (Khi `|angle| <= theta_normal`):**
     + Nếu có lệnh từ Bluetooth (`FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`), cộng thêm offset tốc độ/góc tương ứng.
     + Nếu KHÔNG có lệnh điều khiển (buông tay cầm), tự động khóa mục tiêu vận tốc về 0 để phanh và quay lại trạng thái đứng cân bằng tại chỗ.
   - **Trạng thái `STATE_RECOVERY` (Tự đứng dậy khi `theta_normal < |angle| <= X`):**
     + Khi xe bị ngã nhưng góc nghiêng vẫn nằm trong khoảng nhỏ hơn hoặc bằng `X` so với trục đứng, kích hoạt chuỗi xung lực gia tốc tối đa (Bang-Bang / Swing-up recovery) quay bánh xe cùng chiều nghiêng để hất trọng tâm xe dựng đứng dậy.
     + Khi góc nhỏ hơn `theta_normal`, xóa tích phân PID và chuyển mượt sang `STATE_BALANCING`. Nếu sau một khoảng thời gian timeout (ví dụ 1.5 giây) không dựng dậy được, chuyển về `STATE_IDLE`.
   - **Trạng thái `STATE_IDLE` (Khi `|angle| > X`):**
     + Xe đã đổ hẳn vượt quá góc `X`. Lập tức ngắt PWM về 0, kéo chân `STBY` (`PB0`) xuống LOW để bảo vệ động cơ và pin, xóa bộ nhớ tích phân PID. Chỉ thoát khỏi `STATE_IDLE` khi góc xe quay trở lại vùng `<= X`.
5. `telemetry_uart.c/.h`:
   - Nhận các gói tin điều khiển qua Bluetooth VÀ nhận lệnh cập nhật nóng tham số PID (`SET_PID:kp,ki,kd,skp,ski`) trong RAM.
   - Định kỳ gửi khung dữ liệu Telemetry (`DATA:angle,gyro,pwm,enc_left,enc_right,state\n`) qua UART khi được máy tính yêu cầu thu thập dữ liệu.

#### BƯỚC 4: TẠO DỮ LIỆU GIẢ LẬP & KIỂM TRA KẾT NỐI THU THẬP DỮ LIỆU THỰC TẾ
Viết script `Tools/step4_data_acquisition.py`:
1. Tạo sẵn một bộ sinh dữ liệu giả lập vật lý (`MockDataGenerator`) mô phỏng phản hồi của xe kèm nhiễu cảm biến MPU6050 và vùng chết động cơ, xuất ra `Tools/mock_telemetry.csv`.
2. Tự động quét cổng Serial/USB kết nối với STM32 và gửi bản tin Handshake (`PING\n`) để kiểm tra xem mạch và cảm biến MPU6050 đã được lắp đặt đầy đủ chưa.
3. Phân nhánh xử lý:
   - **Nếu phát hiện mạch STM32 và cảm biến hoạt động đủ:** Kích hoạt chế độ thu thập dữ liệu thực tế trong một cửa sổ ngắn (ví dụ 6 giây) hoặc lấy mẫu theo từng đợt ngắn có kiểm soát (tránh ghi tràn bộ nhớ), lưu kết quả phản hồi thực tế vào `Tools/real_telemetry.csv`.
   - **Nếu KHÔNG cắm mạch hoặc thiếu cảm biến:** Tự động fallback sang sử dụng dữ liệu từ `Tools/mock_telemetry.csv` để phục vụ quy trình kiểm thử và tối ưu ở Bước 5.

#### BƯỚC 5: VÒNG LẶP HIỆU CHỈNH PID CHO ĐẾN KHI XE ĐẠT TRẠNG THÁI CÂN BẰNG
Viết script `Tools/step5_auto_tune_pid.py`:
1. Đọc dữ liệu từ Bước 4 (`real_telemetry.csv` nếu có mạch, hoặc `mock_telemetry.csv` nếu chạy giả lập).
2. Sử dụng thuật toán Nhận dạng hệ thống (System Identification - mô hình ARX / Least Squares) để khớp hàm truyền thực tế của khung xe từ chuỗi dữ liệu vào/ra (`pwm` -> `angle`, `encoder`).
3. Chạy vòng lặp tối ưu hóa (Optimization Loop sử dụng Nelder-Mead / L-BFGS-B với hàm mục tiêu ITAE + phạt dao động điều khiển):
   - Liên tục tinh chỉnh bộ `(Kp, Ki, Kd)` và đánh giá tiêu chuẩn ổn định: Sai số xác lập góc nghiêng tiến về 0 (`|angle_error| < 1.0 độ`), độ vọt lố thấp và không bị dao động tự kích.
   - Lặp lại việc hiệu chỉnh cho đến khi thỏa mãn điều kiện tự cân bằng ổn định thì dừng lại.
4. Lưu bộ thông số tối ưu cuối cùng vào `Tools/computed_gains.json`, cập nhật lại file `Modules/Inc/pid_AutoConfig.h`, đồng thời (nếu đang cắm mạch) gửi lệnh `SET_PID:...` qua UART để cập nhật trực tiếp vào STM32.

#### BƯỚC 6: KIỂM THỬ ĐIỀU KHIỂN TỰ ĐỘNG KHÔNG CẦN BLUETOOTH
Viết script `Tools/step6_test_control_sequence.py`:
1. Cho phép người dùng chạy độc lập để kiểm thử nhanh khả năng di chuyển và quay về cân bằng của xe mà không cần gắn module Bluetooth hay dùng điện thoại.
2. Cho phép truyền vào một chuỗi kịch bản điều khiển định sẵn, ví dụ:
   `[("BALANCE", 2.0), ("FORWARD", 1.5), ("STOP_AND_BALANCE", 2.0), ("BACKWARD", 1.5), ("TURN_LEFT", 1.0), ("TURN_RIGHT", 1.0), ("STOP_AND_BALANCE", 3.0)]`
3. Nếu đang cắm STM32 qua cáp USB-UART, gửi trực tiếp chuỗi lệnh này xuống mạch và vẽ đồ thị/in báo cáo phản hồi theo thời gian thực. Nếu không cắm mạch, chạy chuỗi lệnh này trực tiếp trên bộ mô phỏng vật lý (sử dụng bộ PID đã tinh chỉnh ở Bước 5) và xuất báo cáo đánh giá xem xe có giữ được thăng bằng khi di chuyển và tự phanh đứng im khi ngừng lệnh hay không.
#### BƯỚC 7: ĐÓNG GÓI PROJECT VÀ TÀI LIỆU HÓA (PACKAGING & DOCUMENTATION)

1. TẠO CÁC SCRIPT CHẠY TỰ ĐỘNG ĐA NỀN TẢNG:
   - Cho Linux/macOS:
     + `setup.sh`: Tự động kiểm tra Python 3, tạo môi trường ảo `venv`, kích hoạt và cài đặt toàn bộ thư viện từ `Tools/requirements.txt`.
     + `build_firmware.sh`: Tự động tạo thư mục `build/`, chạy `cmake ..` và `make -j$(nproc)` để biên dịch mã nguồn C/C++ STM32 ra file `.elf` và `.bin`.
     + `run_pipeline.sh`: Script điều phối cho phép chạy nhanh từng bước từ B2 đến B6 hoặc chạy toàn bộ quy trình kiểm thử.
   - Cho Windows:
     + `setup.bat`: Tự động kiểm tra Python, tạo `venv`, kích hoạt `venv\Scripts\activate.bat` và cài đặt `requirements.txt`.
     + `build_firmware.bat`: Kiểm tra CMake và ARM GCC Toolchain trên Windows, tự động build mã nguồn STM32.
     + `run_pipeline.bat`: Menu tương tác dòng lệnh để người dùng chọn chạy Bước 2, Bước 4, Bước 5 hoặc Bước 6.

2. CẤU HÌNH FILE `.gitignore`:
   - Tạo file `.gitignore` ở thư mục gốc của repository với các quy tắc:
     + Bỏ qua thư mục `/self_balancing_car/prompt/` và `prompt/`.
     + Bỏ qua các file build và nhị phân: `build/`, `cmake-build-*/`, `*.o`, `*.d`, `*.map`.
     + Bỏ qua môi trường ảo và cache Python: `venv/`, `.venv/`, `__pycache__/`, `*.pyc`.
     + Bỏ qua các file log tạm thời sinh ra trong quá trình test: `Tools/*.csv` (giữ lại file template hoặc mock mẫu).
     + Bỏ qua cấu hình riêng của IDE: `.vscode/`, `.idea/`, `*.ioc.bak`.

3. VIẾT TÀI LIỆU DỰ ÁN `README.md`:
   - Viết file `README.md` chuyên nghiệp bằng tiếng Việt gồm các mục:
     + **Giới thiệu**: Mục tiêu hệ thống xe 2 bánh tự cân bằng, các tính năng chính (Cascade PID, tự đứng dậy khi nghiêng `<= X`, dừng an toàn khi đổ, điều khiển Bluetooth).
     + **Sơ đồ đấu nối phần cứng (Wiring Diagram)**: Bảng tra cứu chân STM32F401CCU6 nối với MPU6050, TB6612, 2 Encoder GA25 và Bluetooth HC-05 (khớp chuẩn 100% với cấu hình CubeMX).
     + **Yêu cầu môi trường**: Danh sách các công cụ cần cài trên máy tính (ARM GNU Toolchain, CMake, Python 3.8+).
     + **Hướng dẫn sử dụng trên Linux**: Các lệnh từ clone repo, chạy `setup.sh`, build firmware và chạy script hiệu chỉnh PID.
     + **Hướng dẫn sử dụng trên Windows**: Các bước tương ứng bằng file `.bat` và cách kiểm tra nhận cổng COM khi cắm STM32.
     + **Quy trình tinh chỉnh (Tuning Workflow)**: Hướng dẫn người dùng cách sửa thông số trong `Tools/robot_config.yaml` khi thay khung xe mới để hệ thống tự động sinh bộ PID mới mà không cần tính toán thủ công.


---
**YÊU CẦU KỸ THUẬT BẮT BUỘC:**
- Viết toàn bộ mã nguồn thực thi đầy đủ cho tất cả các file C/C++ và Python, KHÔNG viết code giả (pseudo-code) hoặc bỏ trống hàm với comment `// TODO`.
- Sau khi viết xong, tự động chạy lệnh `cmake` và `make` (hoặc `cmake --build`) để kiểm tra firmware STM32 biên dịch thành công 100% không có lỗi cú pháp, sau đó chạy thử tuần tự các script Python từ Bước 2 đến Bước 6 để xác nhận toàn bộ pipeline hoạt động trơn tru.
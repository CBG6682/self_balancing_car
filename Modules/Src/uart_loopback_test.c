/**
 * ==============================================================================
 * @file    uart_loopback_test.c
 * @brief   Chương trình kiểm tra Loopback cổng UART1 (PA9 nối PA10) trên STM32F401.
 * ==============================================================================
 */

#include "uart_loopback_test.h"
#include "telemetry_uart.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

#define LOOPBACK_RX_BUF_SIZE 64

static char s_rx_line[LOOPBACK_RX_BUF_SIZE];
static volatile uint8_t s_rx_idx = 0;
static volatile uint8_t s_new_line_flag = 0;
static char s_completed_line[LOOPBACK_RX_BUF_SIZE];

static void Loopback_RxHook(uint8_t byte)
{
    if (byte == '\n' || byte == '\r') {
        if (s_rx_idx > 0) {
            s_rx_line[s_rx_idx] = '\0';
            strncpy(s_completed_line, s_rx_line, sizeof(s_completed_line) - 1);
            s_completed_line[sizeof(s_completed_line) - 1] = '\0';
            s_new_line_flag = 1;
            s_rx_idx = 0;
        }
    } else {
        if (s_rx_idx < sizeof(s_rx_line) - 1) {
            s_rx_line[s_rx_idx++] = (char)byte;
        } else {
            s_rx_idx = 0;
        }
    }
}

void UART_Loopback_Test_Run(void)
{
    // 1. Đăng ký hook nhận byte UART1 trực tiếp
    Telemetry_RegisterRxHook(Loopback_RxHook);
    Telemetry_StartReceiveIT();

    // 2. Khởi tạo LED PC13 (Ban đầu tắt LED)
    Board_LED_Init();
    Board_LED_Set(0); // 0 = OFF

    uint32_t last_tx_tick = 0;
    volatile uint32_t pass_count = 0;
    volatile uint32_t fail_count = 0;
    uint8_t waiting_ack = 0;
    uint32_t wait_start_tick = 0;

    // Chuỗi kiểm tra chuẩn
    const char ping_str[] = "PING\n";

    while (1) {
        uint32_t now = HAL_GetTick();

        // ---------------------------------------------------------------------
        // 1. Kiểm tra gói tin nhận về từ chân RX (PA10)
        // ---------------------------------------------------------------------
        if (s_new_line_flag) {
            s_new_line_flag = 0;

            // Trường hợp A: Tự kiểm tra nối tắt PA9 sang PA10 (Self-Loopback)
            if (strcmp(s_completed_line, "PING") == 0) {
                if (waiting_ack) {
                    waiting_ack = 0; // Nhận đúng gói PING tự phát
                    pass_count++;
                    Board_LED_Toggle(); // Đèn LED PC13 chớp nháy báo PASS
                } else {
                    // Nếu nhận PING từ bên ngoài (ESP32/PC): Trả về TOKEN ĐỘC NHẤT
                    // để máy tính phân biệt với hiện tượng ESP32 tự dội ngược
                    const char ack_str[] = "PONG_FROM_STM32_HARDWARE_OK\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)ack_str, strlen(ack_str), 20);
                }
            }
            // Trường hợp B: Nhận lệnh Challenge từ script máy tính
            else if (strcmp(s_completed_line, "PING_STM32_CHALLENGE") == 0) {
                const char ack_str[] = "PONG_FROM_STM32_HARDWARE_OK\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)ack_str, strlen(ack_str), 20);
                pass_count++;
                Board_LED_Toggle();
            } else {
                // Nhận tín hiệu rác hoặc không hợp lệ
                fail_count++;
                waiting_ack = 0;
                Board_LED_Set(0); // Tắt LED báo lỗi
            }
        }

        // ---------------------------------------------------------------------
        // 2. Kiểm tra Timeout (Nếu phát PING mà sau 60ms không nhận lại được)
        // ---------------------------------------------------------------------
        if (waiting_ack && (now - wait_start_tick > 60)) {
            // Không nhận được tín hiệu dội về -> Dây nối PA9-PA10 bị rút hoặc hở!
            waiting_ack = 0;
            fail_count++;
            Board_LED_Set(0); // Tắt đèn LED khi mất kết nối
        }

        // ---------------------------------------------------------------------
        // 3. Định kỳ mỗi 200ms: Tự động phát "PING\n" qua chân TX (PA9)
        // ---------------------------------------------------------------------
        if (!waiting_ack && (now - last_tx_tick >= 200)) {
            last_tx_tick = now;
            waiting_ack = 1;
            wait_start_tick = now;

            // Xóa bộ đệm dòng trước khi phát
            s_rx_idx = 0;

            // Phát PING qua chân PA9
            HAL_UART_Transmit(&huart1, (uint8_t*)ping_str, strlen(ping_str), 20);
        }

        (void)pass_count;
        (void)fail_count;
        HAL_Delay(5);
    }
}

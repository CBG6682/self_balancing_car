/**
 * ==============================================================================
 * @file    uart_loopback_test.h
 * @brief   Chương trình kiểm tra Loopback cổng UART1 (PA9 nối PA10) trên STM32F401.
 *          Target: WeAct STM32F401CCU6 BlackPill
 * ==============================================================================
 */

#ifndef UART_LOOPBACK_TEST_H
#define UART_LOOPBACK_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief Khởi động và chạy chu trình kiểm tra Loopback USART1 (PA9/PA10).
 * 
 * Thao tác phần cứng:
 *   - Lấy 1 sợi dây cắm (Jumper Wire), nối thẳng chân PA9 (TX) vào chân PA10 (RX) trên bo STM32.
 * 
 * Cơ chế hoạt động:
 *   - STM32 liên tục phát gói tin "PING\n" qua chân PA9 mỗi 200ms.
 *   - Tín hiệu qua sợi dây cắm đi thẳng vào chân PA10 (RX).
 *   - STM32 kiểm tra xem toàn bộ tín hiệu nhận về có đúng chuẩn "PING" hay không:
 *       ✔ NẾU TOÀN BỘ TÍN HIỆU LÀ "PING": Đèn LED xanh PC13 CHỚP NHÁY LIÊN TỤC (PASS 100%).
 *       ✖ NẾU RÚT DÂY HOẶC BỊ NHIỄU/SAI KÝ TỰ: Đèn LED PC13 TẮT NGÚM (FAIL).
 */
void UART_Loopback_Test_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_LOOPBACK_TEST_H */

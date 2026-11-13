#ifndef __ISP_PROGRAMMER_H
#define __ISP_PROGRAMMER_H

#include "main.h"
#include "bsp_uart.h"


#define U4_BOOT0_HIGH()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define U4_BOOT0_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

#define U4_RESET_HIGH()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define U4_RESET_LOW()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)


#define ISP_ACK            0x79
#define ISP_NACK           0x1F

#define ISP_CMD_SYNC       0x7F
#define ISP_CMD_GET        0x00
#define ISP_CMD_READ       0x11
#define ISP_CMD_GO         0x21
#define ISP_CMD_WRITE      0x31
#define ISP_CMD_ERASE_EXT  0x44
#define ISP_MAX_READ_LEN   256


typedef enum {
    ISP_OK = 0,
    ISP_ERR_SYNC,
    ISP_ERR_NACK,
    ISP_ERR_TIMEOUT,
    ISP_ERR_ERASE,
    ISP_ERR_WRITE,
    ISP_ERR_CHECKSUM,
    ISP_ERR_READ
} ISP_Result_t;



ISP_Result_t ISP_Connect(void);
ISP_Result_t ISP_MassErase(void);
ISP_Result_t ISP_WriteMemory(uint32_t address, uint8_t *data, uint16_t length);
ISP_Result_t ISP_ReadMemory(uint32_t address, uint8_t *data, uint16_t length);
ISP_Result_t ISP_Go(uint32_t address);

/* [2026-11-14] U4 硬复位: BOOT0 拉低 → 复位脉冲(low_ms) → 等应用启动。
 * 参数化脉宽, 便于"第一次短/后续长"的重试策略; 供烧录收尾与调试命令 0x0028 共用。
 * ★现场根因: 复位释放后 NRST 只回到 2.3V (阈值 2.31V) → 芯片卡在临界点,
 *   每次复位的结果会**确定性翻转**(好/坏交替)。所以正确策略是"复位→检查→失败再复位",
 *   而不是固定打两次。根治仍需硬件给 NRST 加 10k 上拉。 */
void U4_HardReset(uint16_t low_ms);

#endif

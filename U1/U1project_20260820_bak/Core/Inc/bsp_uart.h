#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"


extern UART_HandleTypeDef huart1; /* U7 电机板    (8N1, DMA Byte↔Byte) */
extern UART_HandleTypeDef huart2; /* U4 待测板烧录 (9E1, DMA HalfWord→Byte) */

/* [2026-08-20] 恢复 128 (与旧固件一致): 64B 在上位机连发时 U1 主循环来不及消费,
 * U7/U4 FIFO 溢出丢响应 → 上位机卡住。RAM 已通过缓冲复用优化, 128B 无压力。 */
#define UART_BUF_SIZE  128
#define DMA_BUF_SIZE   256

typedef struct {
    uint8_t           buffer[UART_BUF_SIZE];
    volatile uint16_t head;           /* 主程序写指针 (sync 时写入) */
    volatile uint16_t tail;           /* 主程序读指针 (ReadByte 时读取) */
    UART_HandleTypeDef *huart;        /* DMA 同步用 */
    uint8_t           *dma_buf;       /* DMA 环形缓冲 */
    uint16_t           dma_ndtr_init; /* NDTR 初始值 (8N1=512, 9E1=256) */
    uint16_t           dma_xfer_bytes;/* 每次 DMA 传输占用的内存字节数 */
    uint16_t           last_dma_count;/* 上次 __HAL_DMA_GET_COUNTER 值 */
} UART_FIFO_t;

extern UART_FIFO_t u4_fifo;
extern UART_FIFO_t u7_fifo;
extern uint8_t u4_dma_start_ok;   /* 1=USART2 DMA 接收启动成功 (诊断) */
extern uint8_t u4_dma_buf[];      /* USART2 DMA 环形缓冲 (ISP 恢复用) */

uint8_t  BSP_UART_Test(void);       /* 自测: PA9↔PA10环回, 1=通过 */
void     BSP_UART_Init(void);
void     U4_UART_SetMode(uint8_t enable_8n1);   /* 1=8N1(App运行时), 0=8E1(ISP bootloader) */
void     UART_SendByte(UART_HandleTypeDef *huart, uint8_t data);
void     UART_SendArray(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len);
uint8_t  UART_ReadByte(UART_FIFO_t *fifo, uint8_t *pData);
void     UART_ClearBuffer(UART_FIFO_t *fifo);
uint16_t UART_GetUnreadLen(UART_FIFO_t *fifo);
void     BSP_UART_DMA_Sync(UART_FIFO_t *fifo);

#endif

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"

extern UART_HandleTypeDef huart1; /* U1 治具主板 (8N1) */

#define UART_BUF_SIZE  256
#define DMA_BUF_SIZE   512

typedef struct {
    uint8_t           buffer[UART_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    UART_HandleTypeDef *huart;
    uint8_t           *dma_buf;
    uint16_t           dma_ndtr_init;  /* NDTR 初始值 (= DMA_BUF_SIZE for 8N1) */
    uint16_t           dma_xfer_bytes; /* 每次 DMA 传输占用的内存字节数 */
    uint16_t           last_dma_count;
} UART_FIFO_t;

extern UART_FIFO_t uart1_fifo;

void     BSP_UART_Init(void);
void     UART_SendByte(UART_HandleTypeDef *huart, uint8_t data);
void     UART_SendArray(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len);
uint8_t  UART_ReadByte(UART_FIFO_t *fifo, uint8_t *pData);
void     UART_ClearBuffer(UART_FIFO_t *fifo);
uint16_t UART_GetUnreadLen(UART_FIFO_t *fifo);
void     BSP_UART_DMA_Sync(UART_FIFO_t *fifo);

#endif

#include "bsp_uart.h"

/* DMA 环形缓冲 (8N1, 1 传输 = 1 字节) */
static uint8_t u1_dma_buf[DMA_BUF_SIZE];

UART_FIFO_t uart1_fifo = {
    .huart          = &huart1,
    .dma_buf        = u1_dma_buf,
    .dma_ndtr_init  = DMA_BUF_SIZE,
    .dma_xfer_bytes = 1,
};

/**
  * @brief  将 DMA 环形缓冲中的新字节同步到 FIFO
  */
void BSP_UART_DMA_Sync(UART_FIFO_t *fifo)
{
    UART_HandleTypeDef *huart = fifo->huart;
    if (huart == NULL || huart->hdmarx == NULL) return;

    uint16_t cur = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    int32_t  delta;

    if (fifo->last_dma_count >= cur) {
        delta = (int32_t)(fifo->last_dma_count - cur);
    } else {
        delta = (int32_t)(fifo->dma_ndtr_init - cur + fifo->last_dma_count);
    }

    if (delta <= 0) return;

    if ((uint32_t)delta > fifo->dma_ndtr_init) {
        delta = (int32_t)fifo->dma_ndtr_init;
    }

    uint16_t src = ((fifo->dma_ndtr_init - fifo->last_dma_count)
                    * fifo->dma_xfer_bytes) % DMA_BUF_SIZE;
    uint16_t received = (uint16_t)delta;

    while (received--) {
        uint16_t next_head = (fifo->head + 1) % UART_BUF_SIZE;
        if (next_head != fifo->tail) {
            fifo->buffer[fifo->head] = fifo->dma_buf[src];
            fifo->head = next_head;
        }
        src = (src + fifo->dma_xfer_bytes) % DMA_BUF_SIZE;
    }

    fifo->last_dma_count = cur;
}

/**
  * @brief  初始化：清空 FIFO 并启动 DMA 环形接收
  */
void BSP_UART_Init(void)
{
    UART_ClearBuffer(&uart1_fifo);
    HAL_UART_Receive_DMA(&huart1, u1_dma_buf, DMA_BUF_SIZE);

    if (huart1.hdmarx)
        uart1_fifo.last_dma_count = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
}

/**
  * @brief  阻塞式发送单字节
  */
void UART_SendByte(UART_HandleTypeDef *huart, uint8_t data)
{
    HAL_UART_Transmit(huart, &data, 1, 100);
}

/**
  * @brief  阻塞式发送数组
  */
void UART_SendArray(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len)
{
    HAL_UART_Transmit(huart, pData, len, 1000);
}

/**
  * @brief  获取 FIFO 中未读字节数
  */
uint16_t UART_GetUnreadLen(UART_FIFO_t *fifo)
{
    BSP_UART_DMA_Sync(fifo);

    if (fifo->head >= fifo->tail) {
        return fifo->head - fifo->tail;
    } else {
        return UART_BUF_SIZE - fifo->tail + fifo->head;
    }
}

/**
  * @brief  从 FIFO 读取一个字节，空时自动从 DMA 缓冲同步
  * @retval 1=成功, 0=FIFO 为空
  */
uint8_t UART_ReadByte(UART_FIFO_t *fifo, uint8_t *pData)
{
    if (fifo->head == fifo->tail) {
        BSP_UART_DMA_Sync(fifo);
        if (fifo->head == fifo->tail) {
            return 0;
        }
    }
    *pData = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) % UART_BUF_SIZE;
    return 1;
}

/**
  * @brief  清空 FIFO 并重置 DMA 同步状态
  */
void UART_ClearBuffer(UART_FIFO_t *fifo)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    fifo->head = 0;
    fifo->tail = 0;
    if (fifo->huart && fifo->huart->hdmarx) {
        fifo->last_dma_count = __HAL_DMA_GET_COUNTER(fifo->huart->hdmarx);
    }
    if (!primask) {
        __enable_irq();
    }
}

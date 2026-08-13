#include "bsp_uart.h"

/* DMA 环形缓冲 (HALFWORD 传输需 2 字节对齐) */
__ALIGN_BEGIN uint8_t u4_dma_buf[DMA_BUF_SIZE] __ALIGN_END; /* USART2 9E1: NDTR=128, 每传输写 2 字节(低=数据) */
__ALIGN_BEGIN static uint8_t u7_dma_buf[DMA_BUF_SIZE] __ALIGN_END; /* USART1 8N1: NDTR=256, 每传输写 1 字节 */

/* DMA 启动结果 (诊断用): 1=HAL_UART_Receive_DMA 成功 */
uint8_t u4_dma_start_ok = 0;

UART_FIFO_t u4_fifo = {
    .huart          = &huart2,
    .dma_buf        = u4_dma_buf,
    .dma_ndtr_init  = DMA_BUF_SIZE,     /* 8N1: HAL_UART_Receive_DMA Size 传 256 → NDTR=256 */
    .dma_xfer_bytes = 1,                /* Byte→Byte, 每传输写 1 字节 */
};
UART_FIFO_t u7_fifo = {
    .huart          = &huart1,
    .dma_buf        = u7_dma_buf,
    .dma_ndtr_init  = DMA_BUF_SIZE,     /* 8N1: NDTR = Size */
    .dma_xfer_bytes = 1,                /* Byte→Byte, 每次传输 1 字节 */
};

/**
  * @brief  将 DMA 环形缓冲中的新字节同步到 FIFO
  * @note   在 UART_ReadByte 发现 FIFO 空时自动调用；
  *         也可在主循环中主动调用来预填充。
  *         NDTR 语义: 每次递减 1 代表完成 1 次 DMA 传输 =
  *         dma_xfer_bytes 个字节写入 dma_buf。
  */
void BSP_UART_DMA_Sync(UART_FIFO_t *fifo)
{
    UART_HandleTypeDef *huart = fifo->huart;
    if (huart == NULL || huart->hdmarx == NULL) return;

    uint16_t cur = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    int32_t  delta;

    /* 计算自上次 sync 后完成的 DMA 传输次数 */
    if (fifo->last_dma_count >= cur) {
        delta = (int32_t)(fifo->last_dma_count - cur);
    } else {
        /* DMA 回绕: counter 从 0 重载为 dma_ndtr_init */
        delta = (int32_t)(fifo->dma_ndtr_init - cur + fifo->last_dma_count);
    }

    if (delta <= 0) return;

    /* 溢出保护: 若主循环长时间未 sync 导致 DMA 绕了多圈 */
    if ((uint32_t)delta > fifo->dma_ndtr_init) {
        delta = (int32_t)fifo->dma_ndtr_init;
    }

    /* 新字节在 DMA 缓冲中的起始位置 (字节偏移，取模防越界) */
    uint16_t src = ((fifo->dma_ndtr_init - fifo->last_dma_count)
                    * fifo->dma_xfer_bytes) % DMA_BUF_SIZE;
    uint16_t received = (uint16_t)delta;  /* 实际接收的 UART 字符数 */

    while (received--) {
        uint16_t next_head = (fifo->head + 1) % UART_BUF_SIZE;
        if (next_head != fifo->tail) {
            fifo->buffer[fifo->head] = fifo->dma_buf[src];
            fifo->head = next_head;
        }
        /* else: FIFO 满，丢弃 */
        /* 步进 dma_xfer_bytes: 8N1=1, 9E1=2 (跳过校验字节) */
        src = (src + fifo->dma_xfer_bytes) % DMA_BUF_SIZE;
    }

    fifo->last_dma_count = cur;
}

/**
  * @brief  初始化：清空 FIFO 并启动 DMA 环形接收
  * @note   必须在 MX_DMA_Init() 和 MX_USARTx_UART_Init() 之后调用
  */
void BSP_UART_Init(void)
{
    UART_ClearBuffer(&u4_fifo);
    UART_ClearBuffer(&u7_fifo);

    /* 启动 DMA 环形接收
     * USART2 (9E1): Size 传 128 → NDTR=128, 每传输写 2 字节 → 填满 256 字节缓冲
     * USART1 (8N1): Size 传 256 → NDTR=256, 每传输写 1 字节 → 填满 256 字节缓冲
     * 记录 USART2 DMA 启动结果 (诊断用) */
    u4_dma_start_ok = (HAL_UART_Receive_DMA(&huart2, u4_dma_buf, u4_fifo.dma_ndtr_init) == HAL_OK);
    HAL_UART_Receive_DMA(&huart1, u7_dma_buf, DMA_BUF_SIZE);

    /* 读取 DMA 启动后的实际 NDTR 值，与结构体初始值对齐 */
    if (huart2.hdmarx) u4_fifo.last_dma_count = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (huart1.hdmarx) u7_fifo.last_dma_count = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
}

/**
  * @brief  切换 USART2 串口格式: 8N1 (App 运行时) 或 8E1 (ISP bootloader)
  * @param  enable_8n1: 1=8N1(8位无校验), 0=8E1(9位偶校验)
  * @note   [2026-08-12] 新固件 App 用 8N1, G0 系统 bootloader 用 8E1, 需动态切换。
  *         调用前停止 DMA/USART, 调用后重启 DMA 接收。ISP 前切 8E1, PROGRAM 完成后切 8N1。
  */
void U4_UART_SetMode(uint8_t enable_8n1)
{
    extern DMA_HandleTypeDef hdma_usart2_rx;

    HAL_UART_DMAStop(&huart2);
    __HAL_UART_DISABLE(&huart2);
    __HAL_UART_CLEAR_FLAG(&huart2, USART_SR_ORE | USART_SR_FE | USART_SR_NE);
    (void)huart2.Instance->DR;

    if (enable_8n1) {
        huart2.Init.WordLength = UART_WORDLENGTH_8B;
        huart2.Init.Parity     = UART_PARITY_NONE;
        u4_fifo.dma_xfer_bytes = 1;
        u4_fifo.dma_ndtr_init  = DMA_BUF_SIZE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    } else {
        huart2.Init.WordLength = UART_WORDLENGTH_9B;
        huart2.Init.Parity     = UART_PARITY_EVEN;
        u4_fifo.dma_xfer_bytes = 2;
        u4_fifo.dma_ndtr_init  = DMA_BUF_SIZE / 2;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    }

    /* 重新初始化 USART (从 Init 重载 CR1: M/PCE/PS + BRR) */
    if (HAL_UART_Init(&huart2) != HAL_OK) return;

    /* 重新配置 DMA 数据宽度 (8N1=Byte, 8E1=HalfWord) */
    __HAL_DMA_DISABLE(&hdma_usart2_rx);
    HAL_DMA_Init(&hdma_usart2_rx);

    /* 重启 DMA 接收 */
    UART_ClearBuffer(&u4_fifo);
    HAL_UART_Receive_DMA(&huart2, u4_dma_buf, u4_fifo.dma_ndtr_init);
    if (huart2.hdmarx) u4_fifo.last_dma_count = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
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
  * @brief  获取 FIFO 中未读字节数 (含 DMA 缓冲中尚未同步的部分)
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
  * @brief  从 FIFO 读取一个字节
  * @retval 1=成功, 0=FIFO 为空
  * @note   FIFO 空时自动从 DMA 环形缓冲拉取新数据
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
  * @brief  清空 FIFO (关中断保护) 并重置 DMA 同步状态
  * @note   调用后 DMA 缓冲中的旧数据被丢弃，从调用时刻开始接收新数据
  */
void UART_ClearBuffer(UART_FIFO_t *fifo)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    fifo->head = 0;
    fifo->tail = 0;
    /* 重置 DMA 同步基点: 丢弃缓冲中已有的旧数据 */
    if (fifo->huart && fifo->huart->hdmarx) {
        fifo->last_dma_count = __HAL_DMA_GET_COUNTER(fifo->huart->hdmarx);
    }
    if (!primask) {
        __enable_irq();
    }
}

/**
  * @brief  自测: USART1 环回 (PA9↔PA10 需杜邦线短接)
  * @retval 1=通过, 0=失败
  * @note   发 0x55, DMA 收回来比对
  */
uint8_t BSP_UART_Test(void)
{
    /* 启动 DMA 接收 */
    BSP_UART_Init();

    /* 清 FIFO, 丢弃启动阶段的噪声 */
    UART_ClearBuffer(&u7_fifo);
    HAL_Delay(10);

    /* 发送测试字节 0x55 (01010101, 最大翻转率) */
    UART_SendByte(&huart1, 0x55);

    /* 等环回字节到达 (DMA→FIFO 约需 100μs, 给 50ms 足矣) */
    uint32_t tick = HAL_GetTick();
    uint8_t rx;
    while ((HAL_GetTick() - tick) < 50) {
        if (UART_ReadByte(&u7_fifo, &rx)) {
            return (rx == 0x55) ? 1 : 0;
        }
    }
    return 0;   /* 超时未收到 */
}

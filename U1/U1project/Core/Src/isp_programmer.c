#include "isp_programmer.h"
#include "usart.h"

/**
  * @brief  带超时的等待 ACK 函数 (ISP 期间用阻塞接收, 绕开 DMA)
  * @param  timeout_ms: 最大容忍的超时时间
  * @retval ISP_OK (收到0x79), ISP_ERR_NACK (收到0x1F), ISP_ERR_TIMEOUT (超时无响应)
  */
static ISP_Result_t ISP_WaitForACK(uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();
    uint8_t rx_data;

    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (HAL_UART_Receive(&huart2, &rx_data, 1, 50) == HAL_OK) {
            if (rx_data == ISP_ACK) {
                return ISP_OK;
            } else if (rx_data == ISP_NACK) {
                return ISP_ERR_NACK;
            }
        }
    }
    return ISP_ERR_TIMEOUT;
}

/**
  * @brief  向 U4 发送 1 字节指令，并自动附加协议要求的异或校验和
  * @param  cmd: 指令字节
  */
static void ISP_SendCommand(uint8_t cmd) {
    uint8_t packet[2];
    packet[0] = cmd;
    packet[1] = cmd ^ 0xFF;

    /* 清接收残留 (DR + 错误标志), 避免上一命令的响应干扰 */
    __HAL_UART_CLEAR_FLAG(&huart2, USART_SR_ORE | USART_SR_FE | USART_SR_NE);
    (void)huart2.Instance->DR;
    UART_SendArray(&huart2, packet, 2);
}

/**
  * @brief  掌控硬件时序，强行将 U4 拉入 Bootloader 模式并建立连接
  * @note   严格依赖于 PA8(BOOT0) 和 PB15(RESET) 的物理连接
  */
ISP_Result_t ISP_Connect(void) {
    /* ISP 期间停止 DMA 接收, 用阻塞收发绕开 DMA 问题 */
    HAL_UART_DMAStop(&huart2);
    __HAL_UART_CLEAR_FLAG(&huart2, USART_SR_ORE | USART_SR_FE | USART_SR_NE);
    (void)huart2.Instance->DR;   /* 清 RXNE */

    // 物理时序控制
    U4_BOOT0_HIGH();
    U4_RESET_LOW();
    HAL_Delay(10);
    U4_RESET_HIGH();
    HAL_Delay(50);

    UART_SendByte(&huart2, ISP_CMD_SYNC);
    return ISP_WaitForACK(100);
}

/**
  * @brief  对 U4 发起全片擦除 
  * @note   使用 0x44 指令
  */
ISP_Result_t ISP_MassErase(void) {
    ISP_Result_t res;

    // 发送 0x44 扩展擦除指令
    ISP_SendCommand(ISP_CMD_ERASE_EXT);
    res = ISP_WaitForACK(500);
    if (res != ISP_OK) return ISP_ERR_ERASE;

    // G031 擦除格式 (PC USB-TTL 实测): 0x44 + 0xFF 0xFF 0x00 (F1 格式, 无 N-1)
    // 全片擦除: 2字节 0xFFFF + 校验和 0x00
    uint8_t erase_packet[3] = {0xFF, 0xFF, 0x00};
    UART_SendArray(&huart2, erase_packet, 3);

    /* 全片擦除耗时可能 >2s, 加长超时 */
    res = ISP_WaitForACK(10000);
    if (res != ISP_OK) return ISP_ERR_ERASE;

    return ISP_OK;
}

/**
  * @brief  向 U4 物理内存中注入固件数据 (最大支持 256 字节/次)
  * @param  address: 写入的目标绝对基地址 
  * @param  data: 要写入的固件数据片段指针
  * @param  length: 写入长度 (必须在 1 ~ 256 之间)
  */
ISP_Result_t ISP_WriteMemory(uint32_t address, uint8_t *data, uint16_t length) {
    ISP_Result_t res;
    uint8_t packet[5];
    uint8_t checksum = 0;

    /* G0 bootloader 要求: 数据长度 N 必须 8 字节对齐, 地址 8 字节对齐 */
    if (length == 0 || length > 256) return ISP_ERR_WRITE;
    if ((length & 0x07U) != 0) return ISP_ERR_WRITE;
    if ((address & 0x07U) != 0) return ISP_ERR_WRITE;

    ISP_SendCommand(ISP_CMD_WRITE);
    res = ISP_WaitForACK(100);
    if (res != ISP_OK) return ISP_ERR_WRITE;

    packet[0] = (address >> 24) & 0xFF;
    packet[1] = (address >> 16) & 0xFF;
    packet[2] = (address >> 8) & 0xFF;
    packet[3] = address & 0xFF;
    packet[4] = packet[0] ^ packet[1] ^ packet[2] ^ packet[3]; 
    
    UART_ClearBuffer(&u4_fifo);
    UART_SendArray(&huart2, packet, 5);
    res = ISP_WaitForACK(100);
    if (res != ISP_OK) return ISP_ERR_WRITE;

    //写flash通常按页（256字节）写，1字节最大是255，所以st规定用长度-1来表示装入的极限
    uint8_t len_byte = (length - 1) & 0xFF;
    checksum = len_byte; // 校验和初始值包含长度字节

    // 先计算数据校验和，再一次性发送（避免逐字节发送造成的延迟累积）
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }

    UART_ClearBuffer(&u4_fifo);
    UART_SendByte(&huart2, len_byte);
    UART_SendArray(&huart2, data, length);

    // 发送最终计算出的校验和
    UART_SendByte(&huart2, checksum);
    
    res = ISP_WaitForACK(500); 
    if (res != ISP_OK) return ISP_ERR_WRITE;

    return ISP_OK;
}

/**
  * @brief  从 U4 物理内存读取数据 (最大支持 256 字节/次)
  * @param  address: 要读取的目标绝对基地址
  * @param  data: 存放读取数据的缓冲区指针
  * @param  length: 读取长度 (必须在 1 ~ 256 之间)
  * @note   AN2606 Read Memory 指令 (0x11):
  *         0x11 → ACK → addr4B+XOR → ACK → (N-1) → N字节数据 → ACK
  */
ISP_Result_t ISP_ReadMemory(uint32_t address, uint8_t *data, uint16_t length) {
    ISP_Result_t res;
    uint8_t packet[5];

    if (length == 0 || length > ISP_MAX_READ_LEN) return ISP_ERR_READ;

    ISP_SendCommand(ISP_CMD_READ);
    res = ISP_WaitForACK(100);
    if (res != ISP_OK) return ISP_ERR_READ;

    packet[0] = (address >> 24) & 0xFF;
    packet[1] = (address >> 16) & 0xFF;
    packet[2] = (address >> 8) & 0xFF;
    packet[3] = address & 0xFF;
    packet[4] = packet[0] ^ packet[1] ^ packet[2] ^ packet[3];

    UART_ClearBuffer(&u4_fifo);
    UART_SendArray(&huart2, packet, 5);
    res = ISP_WaitForACK(100);
    if (res != ISP_OK) return ISP_ERR_READ;

    /* 发送 (长度-1) + 其 XOR 补码校验字节, bootloader 随后返回 N 字节数据 + ACK
     * [AN3155] Read Memory: 长度字节 N-1 后必须跟校验字节 (N-1)^0xFF,
     *           缺校验字节会导致 bootloader 等待/把下一命令当校验 → NACK (实测 1F) */
    UART_ClearBuffer(&u4_fifo);
    UART_SendByte(&huart2, (length - 1) & 0xFF);
    UART_SendByte(&huart2, (uint8_t)(((length - 1) & 0xFF) ^ 0xFF));

    /* G0 bootloader 对 (N-1)+校验 会回一个 ACK, 必须先消费, 否则被当成数据首位
     * [2026-08-11] 实测: 缺此步导致读回数据整体偏移 1 字节 (含 SN UID 读偏移) */
    res = ISP_WaitForACK(100);
    if (res != ISP_OK) return ISP_ERR_READ;

    /* 轮询读取 N 字节, 带超时 (ISP 期间用阻塞接收) */
    uint32_t tick = HAL_GetTick();
    uint16_t got  = 0;
    while (got < length) {
        if (HAL_UART_Receive(&huart2, &data[got], 1, 50) == HAL_OK) {
            got++;
        } else if ((HAL_GetTick() - tick) > 500) {
            return ISP_ERR_READ;
        }
    }

    /* 数据后的 ACK: G031 bootloader 读内存实测不发送尾随 ACK (4 ACK + 12B 数据)
     * 短超时尝试读一次: 有尾随 ACK 则消费掉, 无则超时视为正常 (数据已读全) */
    res = ISP_WaitForACK(20);
    if (res == ISP_ERR_NACK) return ISP_ERR_READ;

    return ISP_OK;
}

/**
  * @brief  烧录完成，指挥 U4 跳转到指定的地址开始运行新程序
  * @param  address: U4 的固件入口地址 (通常是 0x08000000)
  */
ISP_Result_t ISP_Go(uint32_t address) {
    ISP_Result_t res;
    uint8_t packet[5];

    ISP_SendCommand(ISP_CMD_GO);
    res = ISP_WaitForACK(100);
    if (res != ISP_OK) return ISP_ERR_SYNC;

    packet[0] = (address >> 24) & 0xFF;
    packet[1] = (address >> 16) & 0xFF;
    packet[2] = (address >> 8) & 0xFF;
    packet[3] = address & 0xFF;
    packet[4] = packet[0] ^ packet[1] ^ packet[2] ^ packet[3];

    UART_SendArray(&huart2, packet, 5);
    res = ISP_WaitForACK(100);

    /* 恢复 DMA 接收 (U4 运行时通信用) */
    {
        extern uint8_t u4_dma_buf[];
        extern UART_FIFO_t u4_fifo;
        HAL_UART_Receive_DMA(&huart2, u4_dma_buf, u4_fifo.dma_ndtr_init);
        if (huart2.hdmarx) u4_fifo.last_dma_count = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    }
    return res;
}

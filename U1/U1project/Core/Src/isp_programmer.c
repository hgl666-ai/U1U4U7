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
    /* ★★ [2026-11-15] PB15 已改为【开漏】输出, 时序必须相应放宽 ★★
     * 开漏的 RESET_HIGH 只是"放手(高阻)", NRST 靠 U4 侧的上拉回高,
     * 上升沿由 RC 决定(内部约 40k 上拉 × 板上对地电容), 可能几十毫秒。
     * 原来"释放后只等 50ms 就发 0x7F"会导致芯片还在复位里 → 同步失败 → 烧录直接退出。
     * 现按 M1 治具(UART_FLASH.c)的经验放大:
     *   BOOT0 建立 100ms → 复位低 20ms → 释放后等 300ms 再同步。 */
    U4_BOOT0_HIGH();
    HAL_Delay(100);          /* BOOT0 建立时间 (参考 M1 治具 UF_BOOT_SETUP_MS) */
    U4_RESET_LOW();
    HAL_Delay(20);           /* 复位低脉宽 (参考 M1 治具 UF_RESET_HOLD_MS) */
    U4_RESET_HIGH();         /* 开漏: 释放, 由上拉把 NRST 拉回高 */
    HAL_Delay(300);          /* ★等 NRST 真正升到复位释放电平, 再发同步字节 */

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
  * @brief  [2026-11-14] U4 硬复位: BOOT0 拉低 → 复位脉冲 → 等应用启动
  * @note   供 ①烧录收尾(case 4) 和 ②调试命令 0x0028 共用。
  *
  * 现场问题: 烧录后"每隔一次才进 app"(第1/3次不进, 第2/4次进), 交替出现。
  * 已排除: BOOT0 电平(实测 ≈0V)、NRST 接线(已接)、PB15 引脚复用(flash 用 SPI1 PA5/6/7)。
  * 加长时序(10→50ms)无效 → 说明不是"采样窗口临界"。
  *
  * ★[2026-11-14 诊断版结束] 复位低电平已恢复为正常值 50ms。
  *   实测结论: 复位释放后 NRST 只回到 **2.3V**(不是 3.3V), 而 STM32 复位阈值
  *   恰为 0.7×3.3V = 2.31V → 芯片停在"复位/释放"的临界点上, 于是出现
  *   "每隔一次才进 app"的交替。根因是硬件: NRST 上有下拉/串阻分压,
  *   把 U1 PB15 的推挽高电平吃掉了。
  *   ★根治办法 (硬件): NRST 到 3.3V 加 10k 上拉, 或减小 PB15→NRST 的串阻,
  *     或去掉 NRST 上的下拉/减小对地电容。
  *   软件侧只能做缓解(见 app.c case 4 的双复位), 无法从根本上把 2.3V 抬到 3.3V。
  */
#define U4_RESET_LOW_MS    50U    /* 复位低电平默认保持时间 (ms) */

void U4_HardReset(uint16_t low_ms)
{
    U4_BOOT0_LOW();              /* ① 先确保 BOOT0 = 低 (从 flash 启动) */
    HAL_Delay(20);               /* ② 等 BOOT0 电平稳定 */
    U4_RESET_LOW();              /* ③ 拉低复位 */
    HAL_Delay(low_ms);           /* ④ 复位低电平保持 (调用方指定) */
    U4_RESET_HIGH();             /* ⑤ 释放复位 → U4 从 flash 启动应用 */
    HAL_Delay(300);              /* ⑥ 等应用启动完成 */
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

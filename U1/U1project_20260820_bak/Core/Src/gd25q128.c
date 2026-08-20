#include "gd25q128.h"

/* PA4 = SPI1_NSS, 但在此驱动中用作软件 CS */
#define FLASH_CS_GPIO_Port  GPIOA
#define FLASH_CS_Pin        GPIO_PIN_4
#define CS_ENABLE()  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET)
#define CS_DISABLE() HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET)

/**
  * @brief SPI底层收发一个字节
  */
uint8_t SPI1_SwapByte(uint8_t txData) {
    uint8_t rxData = 0;
    HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, 1000);
    return rxData;
}

/**
  * @brief 读取芯片ID (正常返回 0xC817)
  */
uint16_t GD25Q_ReadID(void) {
    uint16_t temp = 0;
    CS_ENABLE();
    SPI1_SwapByte(GD25Q_DeviceID_Cmd);
    SPI1_SwapByte(0x00);
    SPI1_SwapByte(0x00);
    SPI1_SwapByte(0x00);
    temp = SPI1_SwapByte(0xFF) << 8;
    temp |= SPI1_SwapByte(0xFF);
    CS_DISABLE();
    return temp;
}

/**
  * @brief 自测: 读 Flash ID, 期望 0xC817
  * @retval 1=通过, 0=失败
  */
uint8_t GD25Q_Test(void)
{
    return (GD25Q_ReadID() == 0xC817) ? 1 : 0;
}

/**
  * @brief 写使能 (每次擦除和写入前必须调用)
  */
void GD25Q_WriteEnable(void) {
    CS_ENABLE();
    SPI1_SwapByte(GD25Q_WriteEnable_Cmd);
    CS_DISABLE();
}

/**
  * @brief 等待空闲 (查询状态寄存器的 WIP 位)
  */
uint8_t GD25Q_WaitForWriteEnd(void) {
    uint8_t status = 0;
    uint32_t tickstart = HAL_GetTick();
    CS_ENABLE();
    SPI1_SwapByte(GD25Q_ReadStatusReg1_Cmd);
    do {
        status = SPI1_SwapByte(0xFF);
        if ((HAL_GetTick() - tickstart) > 500) {
            CS_DISABLE();
            return 0;  /* 超时失败 */
        }
    } while ((status & 0x01) == 0x01); // 等待 WIP(Write In Progress) 标志位清零
    CS_DISABLE();
    return 1;  /* 成功 */
}

/**
  * @brief 擦除指定扇区
  * @param SectorAddr: 扇区起始地址
  */
uint8_t GD25Q_EraseSector(uint32_t SectorAddr) {
    uint8_t ok;

    GD25Q_WriteEnable();
    ok = GD25Q_WaitForWriteEnd();
    if (!ok) return 0;

    CS_ENABLE();
    SPI1_SwapByte(GD25Q_SectorErase_Cmd);
    SPI1_SwapByte((uint8_t)((SectorAddr) >> 16));
    SPI1_SwapByte((uint8_t)((SectorAddr) >> 8));
    SPI1_SwapByte((uint8_t)SectorAddr);
    CS_DISABLE();

    return GD25Q_WaitForWriteEnd(); // 等待物理擦除动作完成
}

/**
  * @brief 按页写入数据 
  * @param pBuffer: 数据源指针
  * @param WriteAddr: 写入起始地址
  * @param NumByteToWrite: 写入字节数 
  */
uint8_t GD25Q_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
    /* 支持跨页: 若长度超出当前 256 字节页边界, 自动分多次页编程 */
    while (NumByteToWrite > 0) {
        uint16_t max_bytes = 256 - (WriteAddr & 0xFF);
        uint16_t chunk = (NumByteToWrite > max_bytes) ? max_bytes : NumByteToWrite;

        GD25Q_WriteEnable();
        if (!GD25Q_WaitForWriteEnd()) return 0;

        CS_ENABLE();
        SPI1_SwapByte(GD25Q_PageProgram_Cmd);
        SPI1_SwapByte((uint8_t)((WriteAddr) >> 16));
        SPI1_SwapByte((uint8_t)((WriteAddr) >> 8));
        SPI1_SwapByte((uint8_t)WriteAddr);

        for (uint16_t i = 0; i < chunk; i++) {
            SPI1_SwapByte(pBuffer[i]);
        }
        CS_DISABLE();

        if (!GD25Q_WaitForWriteEnd()) return 0;

        WriteAddr       += chunk;
        pBuffer         += chunk;
        NumByteToWrite  -= chunk;
    }
    return 1;
}

/**
  * @brief 连续读取任意长度数据
  * @param pBuffer: 存放读取数据的指针
  * @param ReadAddr: 读取起始地址
  * @param NumByteToRead: 读取字节数
  */
void GD25Q_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead) {
    CS_ENABLE();
    SPI1_SwapByte(GD25Q_ReadData_Cmd);
    SPI1_SwapByte((uint8_t)((ReadAddr) >> 16));
    SPI1_SwapByte((uint8_t)((ReadAddr) >> 8));
    SPI1_SwapByte((uint8_t)ReadAddr);

    for (uint16_t i = 0; i < NumByteToRead; i++) {
        pBuffer[i] = SPI1_SwapByte(0xFF);
    }
    CS_DISABLE();
}

/**
  * @brief  读状态寄存器1 (0x05)
  * @retval SR1: bit0=WIP, bit1=WEL, bit2-5=BP0-3(块保护), bit6=TB, bit7=SEC
  */
uint8_t GD25Q_ReadSR1(void)
{
    uint8_t sr = 0;
    CS_ENABLE();
    SPI1_SwapByte(GD25Q_ReadStatusReg1_Cmd);
    sr = SPI1_SwapByte(0xFF);
    CS_DISABLE();
    return sr;
}

/**
  * @brief  GD25Q 软件复位 (0x66 + 0x99)
  * @note   上电后首次写入前调用, 确保 Flash 处于干净就绪状态。
  *         实测: 上电后第一次页编程写入能读回但不持久(掉电丢失),
  *         加软件复位后恢复。复位后 Flash 回到默认 3 字节地址模式。
  */
void GD25Q_SoftwareReset(void)
{
    CS_ENABLE();
    SPI1_SwapByte(0x66);   /* Enable Reset */
    CS_DISABLE();
    CS_ENABLE();
    SPI1_SwapByte(0x99);   /* Reset */
    CS_DISABLE();
    HAL_Delay(1);          /* 等复位完成 (约 10-50us, 留余量) */
}

/**
  * @brief  解除块保护: 清状态寄存器1(BP0-3/TB/SEC) 和 状态寄存器2(CMP/QE/SRP)
  * @note   若芯片处于 BP 块保护(低地址区域默认受保护), 擦除/写入会静默失败。
  *         必须在擦除/写入前调用。
  */
void GD25Q_Unprotect(void)
{
    /* 写 SR1 = 0x00: 清 BP0-3, TB, SEC */
    GD25Q_WriteEnable();
    CS_ENABLE();
    SPI1_SwapByte(GD25Q_WriteStatusReg1_Cmd);
    SPI1_SwapByte(0x00);
    CS_DISABLE();
    GD25Q_WaitForWriteEnd();

    /* 写 SR2 = 0x00: 清 CMP, QE, SRP 等 */
    GD25Q_WriteEnable();
    CS_ENABLE();
    SPI1_SwapByte(GD25Q_WriteStatusReg2_Cmd);
    SPI1_SwapByte(0x00);
    CS_DISABLE();
    GD25Q_WaitForWriteEnd();
}

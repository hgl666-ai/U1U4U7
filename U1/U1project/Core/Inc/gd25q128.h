#ifndef __GD25Q128_H
#define __GD25Q128_H

#include "main.h"
#include "spi.h"

// GD25Q128 常用指令集 (与 W25Q 系列完全兼容)
#define GD25Q_WriteEnable_Cmd      0x06  // 写使能
#define GD25Q_ReadStatusReg1_Cmd   0x05  // 读状态寄存器1
#define GD25Q_ReadStatusReg2_Cmd   0x35  // 读状态寄存器2
#define GD25Q_WriteStatusReg1_Cmd  0x01  // 写状态寄存器1
#define GD25Q_WriteStatusReg2_Cmd  0x31  // 写状态寄存器2
#define GD25Q_ReadData_Cmd         0x03  // 读数据
#define GD25Q_PageProgram_Cmd      0x02  // 页编程 (写入)
#define GD25Q_SectorErase_Cmd      0x20  // 扇区擦除 (4KB)
#define GD25Q_DeviceID_Cmd         0x90  // 读制造商/设备ID

// 基础 SPI 接口
uint8_t SPI1_SwapByte(uint8_t txData);
uint16_t GD25Q_ReadID(void);

uint8_t GD25Q_Test(void);       /* 自测: 读ID, 1=通过 */

// 核心读写擦除接口
void GD25Q_WriteEnable(void);
uint8_t GD25Q_WaitForWriteEnd(void);  /* 返回 1=成功, 0=超时 */
uint8_t GD25Q_EraseSector(uint32_t SectorAddr);     /* 返回 1=成功, 0=超时 */
uint8_t GD25Q_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void GD25Q_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);

// 块保护管理
uint8_t GD25Q_ReadSR1(void);        /* 读状态寄存器1 (bit2-5=BP0-3) */
void   GD25Q_Unprotect(void);       /* 清除块保护 (写SR1/SR2=0x00) */

// 初始化
void   GD25Q_SoftwareReset(void);   /* 软件复位 (0x66+0x99), 确保首次写入就绪 */

#endif

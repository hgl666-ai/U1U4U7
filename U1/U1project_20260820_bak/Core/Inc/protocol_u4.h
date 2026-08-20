#ifndef __PROTOCOL_U4_H
#define __PROTOCOL_U4_H

#include "main.h"
#include <stdint.h>

#define U4_MAX_DATA     256
#define U4_RETRY_MAX    3
#define U4_RETRY_DELAY  5

/*===== 命令码 (1 字节) =====*/
#define U4_CMD_ZERO_SENSOR       0x10  /* 传感器归零 */
#define U4_CMD_START_CALIB       0x11  /* 开始校准 */
#define U4_CMD_FINISH_CALIB      0x12  /* 完成校准 */
#define U4_CMD_CANCEL_CALIB      0x13  /* 取消校准 */
#define U4_CMD_READ_FLASH_PARAM  0x14  /* 读取Flash参数并覆盖 */
#define U4_CMD_SAVE_FLASH_PARAM  0x15  /* 保存参数到Flash */
#define U4_CMD_FACTORY_RESET     0x16  /* 恢复出厂默认参数 */
#define U4_CMD_SET_OFFSET        0x17  /* 设置偏移值 (2B u16 um) */
#define U4_CMD_SET_REPORT_PERIOD 0x18  /* 设置报文周期 (2B u16 ms) */
#define U4_CMD_AMS_ZERO          0x20  /* AMS角度清零 */
#define U4_CMD_REPORT            0x30  /* 报文 (U4→U1 自发, 24B ccd_data_t) */
#define U4_CMD_GET_VERSION       0x50  /* 查询版本 (ACK 3B: [主][次][修订]) */
#define U4_CMD_READ_ALL_ADC      0x34  /* 读全部 ADC (QUERY_ADC target=4 用) */
/* 0x33/0x35/0x36 (读单通道ADC/自检/软件复位) 已删除, U4 方未确认且从未使用 */

#define U4_STATUS_OK        0x00
#define U4_STATUS_ERROR     0xFF

#define U4_PROTO_OK             0
#define U4_PROTO_PENDING        1
#define U4_PROTO_ERR_TIMEOUT   -1
#define U4_PROTO_ERR_FRAME     -2
#define U4_PROTO_ERR_CHKSUM    -3
#define U4_PROTO_ERR_BUSY      -4
#define U4_PROTO_ERR_DEVICE    -5   /* 设备已应答, 但返回非 0 状态字节 [2026-08-17] */

/*===== 报文数据结构 (U4→U1, CMD 0x30) =====*/
typedef struct {
    int16_t  length;       /* 计算偏移之后的线径值, 单位 um */
    int16_t  raw_length;   /* 原始线径值, 单位 um */
    int16_t  position;     /* 当前磁条位置, 单位 um, 范围 0-2000 */
    uint16_t vo1;          /* VCP1615-V1: 当前 ADC 值 */
    uint16_t vo2;          /* VCP1615-V2: 当前 ADC 值 */
    uint16_t min_vo1;      /* VCP1615-V1: 测量 ADC 最小值 */
    uint16_t max_vo1;      /* VCP1615-V1: 测量 ADC 最大值 */
    uint16_t min_vo2;      /* VCP1615-V2: 测量 ADC 最小值 */
    uint16_t max_vo2;      /* VCP1615-V2: 测量 ADC 最大值 */
    int32_t  angle_acc;    /* ams5600 累计角度 */
    uint16_t ams_adc;      /* ams5600 当前 ADC 值 */
} U4_ReportData;

/* 最新报文，由 U4_Proto_Run 自动更新 */
extern U4_ReportData u4_report;
extern uint8_t       u4_report_fresh;  /* 1=有新报文待读, 上层读后清零 */

/* [2026-08-20] U4 在线检测 (报文流 33B/50ms, 超时未收即离线) */
#define U4_OFFLINE_TIMEOUT_MS  500
extern volatile uint32_t u4_report_tick;
uint8_t U4_IsOnline(void);          /* 1=在线 (最近 U4_OFFLINE_TIMEOUT_MS 内有报文) */

void U4_Proto_Init(void);
void U4_Proto_Run(void);
void U4_ScanPause(uint8_t pause);   /* 1=暂停空闲报文扫描 (ISP 期间调用) */

int U4_ZeroSensor(void);
int U4_StartCalib(void);
int U4_FinishCalib(void);
int U4_CancelCalib(void);
int U4_ReadFlashParam(void);
int U4_SaveFlashParam(void);
int U4_FactoryReset(void);
int U4_SetOffset(uint16_t offset_um);
int U4_SetReportPeriod(uint16_t period_ms);
int U4_AmsZero(void);
int U4_GetVersion(uint8_t *major, uint8_t *minor, uint8_t *revision);
int U4_ReadAllADC(uint8_t *buf);   /* buf需16字节, 4通道(ADC1/2/3/AVDD)×4B */

#endif

#ifndef __SN_REPORT_H
#define __SN_REPORT_H

#include "main.h"
#include <stdint.h>

/*===== SN 定义 =====*/
#define SN_UID_BASE    0x1FFF7590U   /* STM32G0 96-bit 唯一芯片 ID 基址 */
#define SN_UID_LEN     12U           /* 96-bit = 12 字节 */
#define SN_FRAME_LEN   16U           /* [版本1B][UID 12B][校验1B][预留2B] */

/* 检测阈值与通道
 * ★[2026-11-15 实测确认] 用【U1 自己的 ADC 通道 CH2】(ADC_CH_ADC2 / PA0 / 命令返回的
 *   "通道号 3") 判断 U4 是否在位。拔板实测 = 1(个位数), 插板实测 = 2065(2000多),
 *   阈值 500/200 两边余量都很足。
 * ★为什么不是 CH3: 通道 4(CH3) 拔板后仍有 1367, 不回落 → 用它会导致标志位永不清除,
 *   表现为"只有第一块板能上报 SN"。
 * ★为什么不用 U4 上报的 ADC 值: 4 路 ADC 经 LM358 跟随器, 输入高阻, U4 下电后
 *   前端电容把最后一次读数记住(DUT 的 CH3 拔板后仍停在 2064), 无法检测拔板。
 *   而 U1 自己读的通道 2 是实时跟随的。 */
#define SN_DET_ADC_CH        ADC_CH_ADC2
#define SN_DET_PRESENT_RAW   500U   /* > 此值认为 U4 在位 (实测 2065) */
#define SN_DET_ABSENT_RAW    200U   /* < 此值认为 U4 已拔出 (实测 1) */
#define SN_DET_DEBOUNCE        4U   /* 连续 N 次确认才算稳定 (防插拔抖动) */
#define SN_DET_POLL_MS       200U   /* 轮询周期 */

/*===== API =====*/
void SN_Report_Init(void);            /* 初始化去重状态 (app 启动时调用) */
void SN_Report_OnProgram(void);       /* PROGRAM 握手成功后调用: 读UID→去重→上传SN */
void SN_SendUidFrame(const uint8_t *uid);   /* [2026-11-15] 公开: 直接发一条 0x0024 自发帧 */
void SN_AutoPoll(void);               /* [2026-11-15] 主循环调用: 检测 U4 插入并自动上报 SN */

#endif

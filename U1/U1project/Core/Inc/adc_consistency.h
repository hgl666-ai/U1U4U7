#ifndef __ADC_CONSISTENCY_H
#define __ADC_CONSISTENCY_H

#include "main.h"
#include <stdint.h>

/* ADC 一致性校验 (CMD 0x0020 LEN=2 全芯片模式): U1/U4/U7 三芯片比对 */

/* [2026-11-15] U4 报文"新鲜度"阈值: 超过这么久没收到 0x30 报文, 就认为 U4 已掉线,
 * 其读数按无效处理(status=0), 不再把 RAM 里的旧值当成测量结果。
 * 默认报文周期 50ms, 取 400ms = 连续 8 帧没来。
 * ★若上位机用 0x0018 把报文周期改到 >400ms, 这里必须同步放大, 否则会误判 U4 掉线。 */
#define ADC_CONSIST_U4_FRESH_MS   400U

void     ADC_Consistency_Init(void);          /* 复位状态 */
uint8_t  ADC_Consistency_Start(uint8_t threshold); /* 0=已启动, 1=BUSY */
void     ADC_Consistency_Run(void);           /* APP_Run 周期推进 */
uint8_t  ADC_Consistency_IsBusy(void);        /* 1=进行中 */

#endif

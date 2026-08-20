#ifndef __ADC_CONSISTENCY_H
#define __ADC_CONSISTENCY_H

#include "main.h"
#include <stdint.h>

/* ADC 一致性校验 (CMD 0x0020 LEN=2 全芯片模式): U1/U4/U7 三芯片比对 */

void     ADC_Consistency_Init(void);          /* 复位状态 */
uint8_t  ADC_Consistency_Start(uint8_t threshold); /* 0=已启动, 1=BUSY */
void     ADC_Consistency_Run(void);           /* APP_Run 周期推进 */
uint8_t  ADC_Consistency_IsBusy(void);        /* 1=进行中 */

#endif

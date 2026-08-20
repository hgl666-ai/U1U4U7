#ifndef __ADC_CALIB_H
#define __ADC_CALIB_H

#include "main.h"
#include <stdint.h>

/* ADC 极值自动校准 (CMD 0x0010): 电机往复 + ADC 极值采集 */

void     ADC_Calib_Init(void);              /* 复位状态 */
uint8_t  ADC_Calib_Start(void);             /* 0=已启动, 1=BUSY */
void     ADC_Calib_Run(void);               /* APP_Run 周期推进 */
uint8_t  ADC_Calib_IsBusy(void);            /* 1=进行中 */

#endif

#ifndef __DIAM_CALIB_H
#define __DIAM_CALIB_H

#include "main.h"
#include <stdint.h>

/* 测径精度校准 (CMD 0x0011/0x0012)
 * 电机开环转动预设步数(对应1/1.5/2mm), 读 DUT 实测直径, 判定 |Dactual-N|≤50um */

#define DIAM_CALIB_POS_COUNT        3U      /* 测试位置数 */
#define DIAM_CALIB_TOLERANCE_UM     50U     /* 容忍误差 0.05mm */

#define DIAM_CALIB_STEPS_PER_REV    3200U   /* 电机 1 圈 = 200步 × 16细分 */
#define DIAM_CALIB_OUTPUT_CIRC_UM   15500U  /* 输出轴周长 15.5mm = 15500um */

#define DIAM_CALIB_MOTOR_DELAY_MS   500U    /* 电机到位等待 */
#define DIAM_CALIB_DUT_TIMEOUT_MS   1000U   /* 等 DUT 报文超时 */
#define DIAM_CALIB_MOTOR_ID         0U      /* 电机编号 */
#define DIAM_CALIB_SUMMARY_LEN      25U     /* 汇总: [总判定1] + 3×[8] */

void     DiamCalib_Init(void);             /* 复位状态 */
uint8_t  DiamCalib_Start(void);            /* 0=已启动, 1=BUSY */
void     DiamCalib_Run(void);              /* APP_Run 周期推进 */
uint8_t  DiamCalib_IsBusy(void);           /* 1=进行中 */

#endif

#ifndef __DIAM_CALIB_H
#define __DIAM_CALIB_H

#include "main.h"
#include <stdint.h>

/* 测径精度校准 (CMD 0x0011/0x0012)
 * [2026-08-25] 流程重写:
 *   电机回零(U7 0x29) → U4归零(0x10) → 设偏移0(0x17) →
 *   CCW 增量移动 1/1.5/2mm → 动态等到位 → 100ms 稳定延迟 →
 *   等新报文读直径 → 判定 |Dactual-N|≤50um → 25B 汇总 ACK
 * 方向: 回零在限位端点, 移动必须反向进入行程区 (2026-08-22 经验)
 * ★[2026-11-13] 下文"CW/CCW"是**逻辑方向**。换新丝杆电机后物理方向与旧电机相反,
 *   已在 U7 侧用 MOTOR_DIR_INVERT (bsp_motor.h) 统一取反 ——
 *   U1 发 CCW 仍表示"离开限位、进入行程区", 无需改动。 */

#define DIAM_CALIB_POS_COUNT        3U      /* 测试位置数 */
#define DIAM_CALIB_TOLERANCE_UM     50U     /* 容忍误差 0.05mm */

#define DIAM_CALIB_STEPS_PER_REV    3200U   /* 电机 1 圈 = 200步 × 16细分 */
#define DIAM_CALIB_OUTPUT_CIRC_UM   1000U   /* [2026-11-13] ★改为丝杆导程 (原 15500=旋转机构周长)
                                             * 新电机: 20 固定轴步进, 导程 1mm, 有效行程 12.7mm
                                             * → 1 圈 = 1000um 直线 → 3.2 步/um, 1mm = 3200 步
                                             * (旧电机 1 圈 = 15500um, 1mm 仅 206 步, 差 15.5 倍) */

#define DIAM_CALIB_MOTOR_DELAY_MS   500U    /* 电机到位基础等待 (实际 = 500 + 步数×1ms) */
#define DIAM_CALIB_READ_DELAY_MS    100U    /* 移动到位后、读直径前稳定延迟 */
#define DIAM_CALIB_DUT_TIMEOUT_MS   1000U   /* 等 DUT 报文超时 */
#define DIAM_CALIB_MOTOR_ID         0U      /* 电机编号 */
#define DIAM_CALIB_SUMMARY_LEN      25U     /* 汇总: [总判定1] + 3×[8] */

void     DiamCalib_Init(void);             /* 复位状态 */
uint8_t  DiamCalib_Start(void);            /* 0=已启动, 1=BUSY */
void     DiamCalib_Run(void);              /* APP_Run 周期推进 */
uint8_t  DiamCalib_IsBusy(void);           /* 1=进行中 */

#endif

#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"


#define MOTOR_ID    0

/* 方向 */
#define MOTOR_DIR_CW   0
#define MOTOR_DIR_CCW  1

/* 速度预设 */
#define MOTOR_SPEED_FAST  10000   /* 10kHz 高速 */
#define MOTOR_SPEED_SLOW   2000   /*  2kHz 低速 */

/* 默认微步 1/8 (200×8=1600 脉冲/圈)，可通过 UART 修改 */

void     BSP_Motor_Init(void);
void     BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz);
void     BSP_Motor_Stop(void);
uint8_t  BSP_Motor_IsBusy(void);
uint32_t BSP_Motor_GetDone(void);

#endif

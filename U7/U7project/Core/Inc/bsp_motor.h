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

/* 微步 1/16 (200×16=3200 脉冲/圈)，初始化时由 BSP_TMC_SetMicrostep16() 显式配置，
 * 与 U1 侧 diam_calib.h 的 DIAM_CALIB_STEPS_PER_REV=3200 一致 */

void     BSP_Motor_Init(void);
uint8_t  BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz);  /* 1=已启动 0=拒绝(busy/参数) */
void     BSP_Motor_Stop(void);
uint8_t  BSP_Motor_IsBusy(void);
uint32_t BSP_Motor_GetDone(void);

/**
  * @brief  电机驱动自测: TMC2209 通信 + CW 运动 + CCW 回退
  * @retval 'T'=全通过 '1'=TMC通信失败 '2'=CW超时 '3'=CCW超时
  * @note   阻塞式, 通过 USART1 (PA9, 115200 8N1) 输出进度
  *         上电后直接调用, 不依赖 U1 协议
  */
uint8_t  BSP_Motor_Test(void);

/**
  * @brief  TIM3 独立验证 (纯串口, 无需示波器)
  * @retval 'T'=正常 'E'=异常
  */
uint8_t  BSP_TIM3_Test(void);

#endif

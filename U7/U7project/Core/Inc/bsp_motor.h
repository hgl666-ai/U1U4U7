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

/*===== 梯形加减速参数 [2026-08-19] =====
 * 丢步修复: 直接以目标频率硬启动/硬停 → 平滑起步/减速到位
 * 全程在 ARR 域线性 (f = 32MHz/(ARR+1)), 加减速即 ARR 均匀增减, ISR 内无除法 */
#define MOTOR_F_START_HZ    500U    /* 起步频率: PSC=0 时 16 位 ARR 下限 ≈488Hz, 取 500Hz */
#define MOTOR_FREQ_MAX_HZ  10000U   /* 运行频率上限 (覆盖 SLOW=2000 / FAST=10000) */
#define MOTOR_ACCEL_STEPS   800U    /* 加速段步数 (2000Hz 时 ≈1.9Hz/步) */
#define MOTOR_DECEL_STEPS   800U    /* 减速段步数 */
#define MOTOR_RAMP_GRAN       2U    /* 每 N 步调一档 ARR (2: ISR 开销减半; 1: 最平滑) */
#define MOTOR_MIN_RAMP_STEPS  4U    /* 小于此步数不做加减速, 全程起步低速直跑 */

/* 微步 1/16 (200×16=3200 脉冲/圈)，初始化时由 BSP_TMC_SetMicrostep16() 显式配置，
 * 与 U1 侧 diam_calib.h 的 DIAM_CALIB_STEPS_PER_REV=3200 一致 */

/*===== 回零参数 [2026-08-20] =====
 * IN1(PB9)=零点限位(低电平触发), IN2(PB10)=对侧限位(低电平触发, 保护)。
 * 回零: 电机以 HOME_SPEED 全程低速转动找 IN1, ISR 每步检测, 触发即停在零点。
 * 用 freq=500Hz(==MOTOR_F_START_HZ) 启动 → delta=0 → 无加减速, 全程低速直跑。
 * [2026-08-20 17:3x] ★触发极性实测修正: 硬件外部上拉+开关拉地, 未触发读 1/触发读 0,
 * 判定为**低电平触发** (此前误按"触发变高"实现 → 未触发即误判, 电机不动却回零成功)。
 * [2026-08-21 09:0x] ★回零方向实测修正: 用户连续 3 次回零, 电机 CCW 转满 1600 步
 * (半圈) 从未触发 IN1/IN2 → 开关不在 CCW 行程内, 零点在 **CW 侧** → 方向改 CW。
 * [2026-08-21 10:4x] ★行程实测修正: 用户实测 PB9 恒 3.3V (IN1 未触发) + 无法手动压到开关,
 * CW 走满 1600 步从未触发 → 开关位置可能超出 1600 步行程。加大到 3200 步 (一整圈)
 * 排除"距离不够"; 若 3200 步仍不触发 → 开关未接线/未安装 (需硬件确认)。
 * [2026-08-20 17:1x] 防"电机只走 1 步就停"的间歇性误触发:
 *   ① home_skip 启动保护 (前 N 步不检测, 电荷泵/电平稳定);
 *   ② 去抖: 连续 HOME_DEBOUNCE 步读到触发才确认 (防启动瞬间毛刺)。 */
#define MOTOR_HOME_DIR        MOTOR_DIR_CW    /* 正转找零点 (实测 CCW 半圈无开关) */
#define MOTOR_HOME_SPEED_HZ   500U            /* 回零速度 (与 F_START 相同 → 无加减速) */
#define MOTOR_HOME_MAX_STEPS  3200U           /* 最大步数(一整圈@16微步), 超时保护 → 失败 */
#define MOTOR_HOME_SKIP_FIRST 8U              /* 启动前 8 步不检测限位 (~16ms @500Hz) */
#define MOTOR_HOME_DEBOUNCE   4U              /* 连续 4 步触发才确认 (~8ms @500Hz) */

void     BSP_Motor_Init(void);
uint8_t  BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz);  /* 1=已启动 0=拒绝(busy/参数) */
void     BSP_Motor_Stop(void);
uint8_t  BSP_Motor_IsBusy(void);
uint32_t BSP_Motor_GetDone(void);

/* 回零 API (非阻塞启动, 状态由 HomeDone/HomeFail 查询) */
void     BSP_Motor_Home(void);
uint8_t  BSP_Motor_HomeDone(void);    /* 1=已到零点 */
uint8_t  BSP_Motor_HomeFail(void);    /* 1=失败(超时/撞IN2/启动失败) */

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

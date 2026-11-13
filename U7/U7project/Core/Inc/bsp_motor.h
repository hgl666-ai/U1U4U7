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
 * [2026-11-13] ★换 20mm 固定轴丝杆电机 (导程 1mm, 有效行程 12.7mm) 后重算:
 *   1 圈 = 1000um 直线 → 12.7mm 全行程 = 40640 步 → 上限取 42000。
 *   回零速度 500→2500Hz (0.78mm/s ≈ 47rpm, 远低于电机 600rpm 上限):
 *   42000 步 @500Hz 要 84s (必超看门狗), @2500Hz 约 17s, 看门狗放到 25s。
 *   注意: 该速度 ≠ MOTOR_F_START_HZ, 加减速斜坡会被启用 (500→2500Hz)。
 *   启动保护/去抖是按"步数"计的, 变速后须同比缩放才能维持原物理时长 (16ms / 8ms)。
 * [2026-11-13 15:1x] ★实测: 换电机后回零"转反了" → 同一 DIR 电平下丝杆直线方向与
 *   旧电机相反, 故新增 MOTOR_DIR_INVERT 在 BSP_Motor_Move 里统一取反。见下方定义。
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

/* [2026-11-13] ★★ 电机/丝杆装配方向取反 (实测: 换新电机后回零转反) ★★
 * 现象: 同一 DIR 电平下, 新电机的丝杆直线运动方向与原电机相反
 *       (线圈 A/B 接反 或 丝杆旋向相反 或 电机装在另一侧)。
 * 处理: 在 BSP_Motor_Move 写 DIR 引脚前统一 XOR 本标志 —— 一处取反, 全局生效。
 *
 * ★为什么必须"全局"取反, 而不能只翻回零方向:
 *   系统不变式 = 【回零朝限位开关】+【定位移动朝反方向进入行程区】。
 *   只翻回零 → 回零对了但定位朝开关走 → 顶死限位、丢步, 而且现象隐蔽。
 *   这里统一取反后, 不变式自动保持, U1 侧无需改动。
 *
 * 若改为硬件修正 (把某一相的 A+/A- 或 B+/B- 对调), 请把本值改回 0。
 * 1 = 取反, 0 = 不取反 */
#define MOTOR_DIR_INVERT      1

#define MOTOR_HOME_DIR        MOTOR_DIR_CW    /* 逻辑方向=找零点侧 (物理方向已由 MOTOR_DIR_INVERT 取反) */
#define MOTOR_HOME_SPEED_HZ   1500U           /* [2026-11-13] 4000→2500→1500 (实测 4000 偏快/嗡嗡);
                                               * 0.47mm/s, 42000 步全程约 28s */
#define MOTOR_HOME_MAX_STEPS  42000U          /* [2026-11-13] 3200→42000: 覆盖 12.7mm 全行程(40640)+余量;
                                               * 旧 3200 步在 1mm 导程下只有 1mm, 必然回零超时 */
#define MOTOR_HOME_SKIP_FIRST 40U             /* 前 40 步不检测 (~16ms @2500Hz, 与旧 8步@500Hz 等效) */
#define MOTOR_HOME_DEBOUNCE   20U             /* 连续 20 步触发才确认 (~8ms @2500Hz, 与旧 4步@500Hz 等效) */

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
